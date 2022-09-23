#include "gnss/ntrip_client.hpp"

namespace sgd_hardware_drivers
{

Ntrip_Client::Ntrip_Client(ntrip_opts options)
{
    options_ = options;
}

Ntrip_Client::~Ntrip_Client()
{
    std::cout << "Close connection" << std::endl;
    close(sockfd);
}

void
Ntrip_Client::start_client()
{
    t = std::thread(&Ntrip_Client::update, this);
    t.detach();
}

void Ntrip_Client::update()
{
    err_ = 0;

    int numbytes;
    char buf[MAXDATASIZE];
    struct sockaddr_in their_addr; /* connector's address information */
    struct hostent *he;
    struct servent *se;
    char *b;
    int i;

    // set port and server address
    memset(&their_addr, 0, sizeof(struct sockaddr_in));
    if ((i = strtol(options_.port, &b, 10)) && (!b || !*b))
    {
        their_addr.sin_port = htons(i); // host to network byte order for unsigned short int (linux function)
    }
    else if (!(se = getservbyname(options_.port, 0)))
    {
        fprintf(stderr, "Can't resolve port %s.", options_.port);
        err_ |= 0x01;
        return;
    }
    else
    {
        their_addr.sin_port = se->s_port;
    }

    // set server address and socket
    if (!(he = gethostbyname(options_.server.c_str())))
    {
        fprintf(stderr, "Server name lookup failed for '%s'.\n", options_.server.c_str());
        err_ |= 0x02;
    }
    else if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        err_ |= 0x02;
    }
    else
    {
        their_addr.sin_family = AF_INET;
        their_addr.sin_addr = *((struct in_addr *)he->h_addr);
    }
    if (err_ != 0)  return;

    // build http request
    std::string request("GET /");
    request += options_.mountpnt;   // mountpnt can be an empty string
    request += " HTTP/1.1" + crlf;
    request += "Host: " + options_.server + crlf;
    request += "User-Agent: NTRIP ROS2NtripClient/1.00" + crlf;
    request += "Connection: close" + crlf;
    request += (!options_.mountpnt.empty()) ? ("Authorization: Basic " + encode_str() + crlf) : "";
    request += crlf;
    request += (options_.nmea && !gga.empty()) ? (gga + crlf) : "";

    fprintf(stderr, "Send request to server:\n");
    for (int k = 0; k < request.size() /*&& buf[k] != '\n' && buf[k] != '\r'*/; ++k)
    {
        fprintf(stderr, "%c", isprint(request[k]) ? request[k] : '.');
    }
    fprintf(stderr, "\n");

    // connect to caster
    // Get sourcetable: "GET / HTTP/1.1\r\nHost: <server>\r\nUser-Agent: <agent>/<revision>\r\nConnection: close\r\n"
    if (connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1)
    {
        // connect failed
        perror("connect");
        err_ |= 0x02;
        return;
    }

    // send request
    if (send(sockfd, request.c_str(), (size_t)i, 0) != i)
    {
        // send failed
        printf("Error while sending request");
        err_ |= 0x02;
        return;
    }
    else if (!options_.mountpnt.empty() && options_.mountpnt.front() != '%')
    {
        int k = 0;
        int chunkymode = 0;
        int starttime = time(0);
        int lastout = starttime;
        int totalbytes = 0;
        int chunksize = 0;

        while (!err_ && (numbytes = recv(sockfd, buf, MAXDATASIZE - 1, 0)) > 0)
        {
            fprintf(stderr, "\nRead %i bytes:\n", numbytes);
            // read successful
            if (!k)
            {
                fprintf(stderr, "!%i\n", k);
                buf[numbytes] = 0; /* latest end mark for strstr */
                if (numbytes > 17 &&
                    !strstr(buf, "ICY 200 OK") && /* case 'proxy & ntrip 1.0 caster' */
                    (!strncmp(buf, "HTTP/1.1 200 OK\r\n", 17) ||
                        !strncmp(buf, "HTTP/1.0 200 OK\r\n", 17)))
                {
                    const char *datacheck = "Content-Type: gnss/data\r\n";
                    const char *chunkycheck = "Transfer-Encoding: chunked\r\n";
                    int l = strlen(datacheck) - 1;
                    int j = 0;
                    for (i = 0; j != l && i < numbytes - l; ++i)
                    {
                        for (j = 0; j < l && buf[i + j] == datacheck[j]; ++j)
                            ;
                    }
                    if (i == numbytes - l)
                    {
                        fprintf(stderr, "No 'Content-Type: gnss/data' found\n");
                        err_ |= 0x02;
                    }
                    l = strlen(chunkycheck) - 1;
                    j = 0;
                    for (i = 0; j != l && i < numbytes - l; ++i)
                    {
                        for (j = 0; j < l && buf[i + j] == chunkycheck[j]; ++j)
                            ;
                    }
                    if (i < numbytes - l)
                        chunkymode = 1;
                }
                else if (!strstr(buf, "ICY 200 OK"))
                {
                    fprintf(stderr, "Could not get the requested data: ");
                    for (k = 0; k < numbytes /*&& buf[k] != '\n' && buf[k] != '\r'*/; ++k)
                    {
                        fprintf(stderr, "%c", isprint(buf[k]) ? buf[k] : '.');
                    }
                    fprintf(stderr, "\n");
                    err_ |= 0x02;
                }

                k = 1;
                continue; /* skip old headers for NTRIP1 */
            }

            if (chunkymode)
            {
                fprintf(stderr, "chunkymode = %i\n", chunkymode);
                int cstop = 0;
                int pos = 0;
                while (!err_ && !cstop && pos < numbytes)
                {
                    switch (chunkymode)
                    {
                    case 1: /* reading number starts */
                        chunksize = 0;
                        ++chunkymode; /* no break */
                    case 2:           /* during reading number */
                        i = buf[pos++];
                        if (i >= '0' && i <= '9')
                            chunksize = chunksize * 16 + i - '0';
                        else if (i >= 'a' && i <= 'f')
                            chunksize = chunksize * 16 + i - 'a' + 10;
                        else if (i >= 'A' && i <= 'F')
                            chunksize = chunksize * 16 + i - 'A' + 10;
                        else if (i == '\r')
                            ++chunkymode;
                        else if (i == ';')
                            chunkymode = 5;
                        else
                            cstop = 1;
                        break;
                    case 3: /* scanning for return */
                        if (buf[pos++] == '\n')
                            chunkymode = chunksize ? 4 : 1;
                        else
                            cstop = 1;
                        break;
                    case 4: /* output data */
                        i = numbytes - pos;
                        if (i > chunksize)
                            i = chunksize;
                        fwrite(buf + pos, (size_t)i, 1, stdout);
                        totalbytes += i;
                        chunksize -= i;
                        pos += i;
                        if (!chunksize)
                            chunkymode = 1;
                        break;
                    case 5:
                        if (i == '\r')
                            chunkymode = 3;
                        break;
                    }
                }
                if (cstop)
                {
                    fprintf(stderr, "Error in chunky transfer encoding\n");
                    err_ |= 0x02;
                }
            }
            else
            {
                // save rtcm message
                totalbytes += numbytes;
                rtcmMutex.lock();
                for (k = 0; k < numbytes; ++k)
                {
                    rtcm += buf[k];
                }
                rtcmMutex.unlock();
            }
            fflush(stdout);
            if (totalbytes < 0) /* overflow */
            {
                totalbytes = 0;
                starttime = time(0);
                lastout = starttime;
            }
        }
    }
    else
    {
        while (!(err_ & 0x01) && (numbytes = recv(sockfd, buf, MAXDATASIZE - 1, 0)) > 0)
        {
            //alarm(ALARMTIME);
            fwrite(buf, (size_t)numbytes, 1, stdout);
        }
        err_ |= 0x01;   // stop after print
    }
}

// ### private methods ####

std::string
Ntrip_Client::encode_str()
{
    int fill = 0;
    // build string from username and password
    std::string usrpwd = options_.user;
    usrpwd += ':';
    usrpwd += options_.password;
    while (usrpwd.length() % 3 != 0)
    {
        usrpwd += '\0';
        fill++;
    }

    std::string out;
    unsigned char inbuf[3];
    int i = 0;
    for (char& c : usrpwd)
    {
        inbuf[i%3] = c;
        i++;
        
        if (i != 0 && (i % 3 == 0))
        {
            out += encodingTable[(inbuf[0] & 0xFC) >> 2];
            out += encodingTable[((inbuf[0] & 0x03) << 4) | ((inbuf[1] & 0xF0) >> 4)];
            out += (i >= usrpwd.length() && fill == 2) ? '=' 
                        : encodingTable[((inbuf[1] & 0x0F) << 2) | ((inbuf[2] & 0xC0) >> 6)];
            out += (i >= usrpwd.length() && fill >= 1) ? '=' : encodingTable[inbuf[2] & 0x3F];
        }
    }
    return out;
}

} // namespace sgd_hardware_drivers
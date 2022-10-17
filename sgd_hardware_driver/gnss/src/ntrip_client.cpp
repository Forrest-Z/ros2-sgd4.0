// Copyright 2022 HAW Hamburg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gnss/ntrip_client.hpp"

namespace sgd_hardware_drivers
{

Ntrip_Client::Ntrip_Client(ntrip_opts options)
{
    options_ = options;
}

Ntrip_Client::~Ntrip_Client()
{
    std::cout << "Close connection\n";
    pthread_cancel(thread_handle);
    int ret = close(sockfd);
    std::cout << "Close returned " << ret << std::endl;
}

void
Ntrip_Client::start_client()
{
    std::thread t = std::thread(&Ntrip_Client::update, this);
    thread_handle = t.native_handle();
    t.detach();
}

void Ntrip_Client::update()
{
    while (true)
    {
        err_ = 0;

        std::time_t last_gga_send;
        int numbytes_;
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
        request += options_.mountpnt;
        request += " HTTP/1.0" + crlf;
        request += "User-Agent: NTRIP ROS2NtripClient/1.00" + crlf;
        //request += "User-Agent: NTRIP LefebureNTRIPClient/20131124" + crlf;
        request += "Accept: */*" + crlf;
        request += "Connection: close" + crlf;
        request += (!options_.mountpnt.empty()) ? ("Authorization: Basic " + options_.auth + crlf) : "";
        request += crlf;

        // set time for gga send in the past so that we can send immediately
        last_gga_send = std::time(nullptr) - 10;

        // connect to caster
        if (connect(sockfd, (struct sockaddr *)&their_addr, sizeof(struct sockaddr)) == -1)
        {
            // connect failed
            printf("Error connecting to server\n");
            err_ |= 0x02;
            return;
        }

        // send request
        if (send(sockfd, request.c_str(), request.size(), 0) < 0)
        {
            // send failed
            printf("Error while sending request\n");
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

            // struct for poll
            struct pollfd pfd[1];
            pfd[0].fd = sockfd;
            pfd[0].events = POLLIN;
            
            while (!err_)
            {
                if (poll(pfd, 1, 1000) > 0) // wait for up to 1s for new data
                {
                    // data is ready
                    numbytes_ = recv(sockfd, buf, MAXDATASIZE - 1, 0);

                    // read successful
                    if (!k)
                    {
                        fprintf(stderr, "!%i\n", k);

                        buf[numbytes_] = 0; /* latest end mark for strstr */
                        if (numbytes_ > 17 &&
                            !strstr(buf, "ICY 200 OK") && /* case 'proxy & ntrip 1.0 caster' */
                            (!strncmp(buf, "HTTP/1.1 200 OK\r\n", 17) ||
                                !strncmp(buf, "HTTP/1.0 200 OK\r\n", 17)))
                        {
                            fprintf(stderr, "ICY 200 OK\n");
                            const char *datacheck = "Content-Type: gnss/data\r\n";
                            const char *chunkycheck = "Transfer-Encoding: chunked\r\n";
                            int l = strlen(datacheck) - 1;
                            int j = 0;
                            for (i = 0; j != l && i < numbytes_ - l; ++i)
                            {
                                for (j = 0; j < l && buf[i + j] == datacheck[j]; ++j)
                                    ;
                            }
                            if (i == numbytes_ - l)
                            {
                                fprintf(stderr, "No 'Content-Type: gnss/data' found\n");
                                err_ |= 0x02;
                            }
                            l = strlen(chunkycheck) - 1;
                            j = 0;
                            for (i = 0; j != l && i < numbytes_ - l; ++i)
                            {
                                for (j = 0; j < l && buf[i + j] == chunkycheck[j]; ++j)
                                    ;
                            }
                            if (i < numbytes_ - l)
                                chunkymode = 1;
                        }
                        else if (!strstr(buf, "ICY 200 OK"))
                        {
                            fprintf(stderr, "Could not get the requested data: ");
                            for (k = 0; k < numbytes_ /*&& buf[k] != '\n' && buf[k] != '\r'*/; ++k)
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
                        while (!err_ && !cstop && pos < numbytes_)
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
                                i = numbytes_ - pos;
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
                        totalbytes += numbytes_;
                        rtcmMutex.lock();
                        std::memcpy(rtcm, buf, numbytes_);
                        numbytes = numbytes_;
                        rtcmMutex.unlock();
                    }
                }

                // send gga string
                std::time_t now = std::time(nullptr);   // time in seconds
                if (now - last_gga_send > 8 && options_.nmea)
                {
                    // send gga to server
                    ggaMutex.lock();
                    if (!gga.empty())
                    {
                        if (send(sockfd, gga.c_str(), (size_t)gga.size(), 0) < 0)
                        {
                            // send failed
                            printf("Error while sending gga data");
                        }
                        last_gga_send = now;
                    } 
                    else
                    {
                        fprintf(stderr, "No gga data to send...\n");
                    }                
                    ggaMutex.unlock();
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
            while (!(err_ & 0x01) && (numbytes_ = recv(sockfd, buf, MAXDATASIZE - 1, 0)) > 0)
            {
                fwrite(buf, (size_t)numbytes_, 1, stdout);
            }
            err_ |= 0x01;   // stop after print
        }

        // there was an error, so close file descriptor
        close(sockfd);
    }
    fprintf(stderr, "Terminate NTRIP updater\n");
}

} // namespace sgd_hardware_drivers
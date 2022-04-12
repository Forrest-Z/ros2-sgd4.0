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

#ifndef NAV_SGD__ADR_SERVER_HPP_
#define NAV_SGD__ADR_SERVER_HPP_

#include <string>
#include <unordered_map>

namespace nav_sgd
{
    
class Adr_Server
{
private:
    std::unordered_map<std::string, std::string> adresslist;
public:
    Adr_Server(std::string adr_filename);
    ~Adr_Server();
};

Adr_Server::Adr_Server(std::string adr_filename)
{
    // read and parse file with addresses
}

Adr_Server::~Adr_Server() {}


} // namespace nav_sgd


#endif
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

#ifndef SGD_UTIL__IEEE_754_CONV_HPP_
#define SGD_UTIL__IEEE_754_CONV_HPP_

#include <iostream>
#include <string>
#include <sstream>
#include <bitset>

namespace sgd_util
{

// define datatypes to assure enough precision on different systems
typedef float ieee754_single;
typedef double ieee754_double;
typedef long double ieee754_ldouble;

/**
 * @brief 
 * 
 */
static ieee754_double toDouble(std::string hex, bool swap_endianess = false)
{
    if (swap_endianess)
    {
        std::string hex_;
        for (int i = hex.size()-2; i >= 0; i-=2)
        {
            hex_.append(hex, i, 2);
        }
        hex = hex_;
    }

    // create bitset
    std::bitset<64> b1(std::stoull(hex, nullptr, 16));

    int sign = b1[b1.size()-1];
    // get exponent
    auto exp_b = b1;
    int exp = (exp_b>>=52).to_ulong() - 1023; // - sign * pow(2, 11);

    // get mantissa
    std::string s;
    s.assign(52, '1');
    std::bitset<64> b0{s};
    b1&=b0;
    
    double mant = 1.0;
    for (int i = 0; i < 52; i++)
    {
        mant += b1[i] / pow(2, 52-i);
        //std::cout << "Mantissa: " << mant << " \ti: " << i << " \tb1[i]: " << b1[i] << std::endl;
    }
    double result = pow(-1, sign) * mant * pow(2,exp);
}

} // namespace sgd_util


#endif
/* -*- C++ -*- */
/** @file "SingletonWrapper.tcc"
 * @brief SingletonWrapper implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef In_H_SingletonWrapper
# error "Cannot include tcc file outside of its corresponding header"
#else

#include <typeinfo>

namespace TREX {
  namespace utils {
    
    namespace internal {
      
      template<typename Ty>
      struct swrapper_factory :public sdummy_factory {
        SingletonDummy *create() const {
          return new SingletonWrapper<Ty>;
        }
      };
      
    }
    
    // statics :
    
    template<typename Ty>
    Ty *SingletonWrapper<Ty>::attach() {
      SingletonWrapper<Ty> *
      me = static_cast<SingletonWrapper<Ty> *>(internal::SingletonDummy::attach(name(),
                                                                                internal::swrapper_factory<Ty>()));
      return &(me->m_value);
    }
    
    template<typename Ty>
    void SingletonWrapper<Ty>::detach() {
      internal::SingletonDummy::detach(name());
    }
    
    template<typename Ty>
    std::string SingletonWrapper<Ty>::name() {
      return typeid(Ty).name();
    }
    
    template<typename Ty>
    void SingletonWrapper<Ty>::disable_server() {
      internal::SingletonDummy::disable();
    }

    
    // structors :
    
    template<typename Ty>
    SingletonWrapper<Ty>::SingletonWrapper() {}
    
    template<typename Ty>
    SingletonWrapper<Ty>::~SingletonWrapper() {}
    
  }
}

#endif

// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef CYBERDOG_DEBUG__BACKTRACE_HPP_
#define CYBERDOG_DEBUG__BACKTRACE_HPP_
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace debug
{
static void dump(void)
{
#define BACKTRACE_SIZE   16
  int j, nptrs;
  void * buffer[BACKTRACE_SIZE];
  char ** strings;

  nptrs = backtrace(buffer, BACKTRACE_SIZE);
  strings = backtrace_symbols(buffer, nptrs);

  if (strings == NULL) {
    ERROR("Backtrace_symbols error!");
    exit(EXIT_FAILURE);
  }

  for (j = 0; j < nptrs; j++) {
    ERROR("\t[%02d] %s\n", j, strings[j]);
  }

  free(strings);
}

static void signal_handler(int signo)
{
  ERROR("=========>>>catch signal %d <<<=========\n", signo);
  ERROR("Backtrace start...\n");
  dump();
  ERROR("Backtrace end...\n");

  signal(signo, SIG_DFL);
}

static void register_signal()
{
  // You can add more signal here.
  signal(SIGSEGV, signal_handler);
  signal(SIGABRT, signal_handler);
}
}  // namespace debug
}  // namespace cyberdog
#endif  // CYBERDOG_DEBUG__BACKTRACE_HPP_

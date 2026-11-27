#pragma once
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <poll.h>
#include <unistd.h>

namespace handyman_stream {
enum class Result { complete, closed, truncated, io_error, stopped, invalid_length, invalid_terminator };

template<class Running>
Result exact(int fd, char* dst, std::size_t count, Running running) {
  std::size_t offset=0;
  while(offset<count) {
    if(!running()) return Result::stopped;
    pollfd p{fd,POLLIN,0};
    const int ready=::poll(&p,1,100);
    if(ready==0) continue; // Preserve offset across arbitrary pauses.
    if(ready<0) { if(errno==EINTR) continue; return Result::io_error; }
    if(p.revents&POLLNVAL) return Result::io_error;
    const auto n=::read(fd,dst+offset,count-offset);
    if(n==0) return offset ? Result::truncated : Result::closed;
    if(n<0) {
      if(errno==EINTR || errno==EAGAIN || errno==EWOULDBLOCK) continue;
      return Result::io_error;
    }
    offset+=static_cast<std::size_t>(n);
  }
  return Result::complete;
}

template<class Running>
Result frame(int fd, char* dst, std::size_t capacity, std::size_t& size, Running running) {
  size=0;
  if(capacity<5) return Result::invalid_length;
  auto result=exact(fd,dst,4,running);
  if(result!=Result::complete) return result;
  const auto* b=reinterpret_cast<const unsigned char*>(dst);
  const uint32_t length=uint32_t(b[0])|(uint32_t(b[1])<<8)|(uint32_t(b[2])<<16)|(uint32_t(b[3])<<24);
  if(length<5 || length>capacity || length>0x7fffffffU) return Result::invalid_length;
  result=exact(fd,dst+4,length-4,running);
  if(result==Result::closed) return Result::truncated;
  if(result!=Result::complete) return result;
  if(dst[length-1]!=0) return Result::invalid_terminator;
  size=length;
  return Result::complete;
}
} // namespace handyman_stream

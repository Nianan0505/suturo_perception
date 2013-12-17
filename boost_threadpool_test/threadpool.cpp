#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <iostream>

#include "threadpool.h"

boost::mutex mutex;

int fib(int x) {
    if (x == 0) return 0;
    if (x == 1) return 1;
    return fib(x-1)+fib(x-2);
}

void doSomething(int value)
{
  int res = fib(value);
  boost::lock_guard<boost::mutex> lock(mutex);
  std::cout << "doSomething(): " << fib(value) << std::endl;
}

void doSomethingElse(int value)
{
  boost::lock_guard<boost::mutex> lock(mutex); 
  std::cout << "doSomethingElse(): " << value+value << std::endl;
}

int main(int argc, char** argv)
{
  // create asio ioservice and threadgroup for the pool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  //boost::asio::io_service::work work(ioService);
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < 2; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService));  
  }

  // post work to the ioservice
  {
    ioService.post(boost::bind(doSomething, 40));
    ioService.post(boost::bind(doSomethingElse, 3));
    ioService.post(boost::bind(doSomething, 40));
    ioService.post(boost::bind(doSomethingElse, 3));
    ioService.post(boost::bind(doSomething, 40));
    ioService.post(boost::bind(doSomethingElse, 3));  
  }
  
  // run the tasks and return, if all queued work is finished
  work.reset();
  ioService.run();
  //ioService.reset(); //needs to be done before run can be executed again
  
  // join all threads of the group
  threadpool.join_all();
}

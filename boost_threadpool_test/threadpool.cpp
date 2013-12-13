#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <string>

boost::mutex mutex;

int fib(int x) {
    if (x == 0) return 0;
    if (x == 1) return 1;
    return fib(x-1)+fib(x-2);
}

void doSomething(int value)
{
  mutex.lock();
  std::cout << "doSomething(): " << fib(value) << std::endl;
  mutex.unlock();
}

void doSomethingElse(int value)
{
  mutex.lock();
  std::cout << "doSomethingElse(): " << value+value << std::endl;
  mutex.unlock();
}

int main(int argc, char** argv)
{
  // create asio ioservice and threadgroup for the pool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;

  // Add worker threads to threadpool
  threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
  );
  threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
  );

 
  // Work object can be used to block the ioservice. it will not return and has to be stopped manually
  //boost::asio::io_service::work work(ioService);

  // post work to the ioservice
  ioService.post(boost::bind(doSomething, 42));
  ioService.post(boost::bind(doSomethingElse, 3));
  ioService.post(boost::bind(doSomething, 42));
  ioService.post(boost::bind(doSomethingElse, 3));

  // this wait is needed, if a work object is used. otherwise the tasks may not be in state 'working'
  // and the service will stop immediately.
  //boost::this_thread::sleep(boost::posix_time::microseconds(10));
  //ioService.stop();
  
  // run the tasks and return, if all queued work is finished
  ioService.run();
  //ioService.reset(); //needs to be done before run can be executed again
  
  // join all threads of the group
  threadpool.join_all();

}

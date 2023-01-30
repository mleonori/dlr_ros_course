
#include <iostream>
#include <boost/asio.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

// string read_(tcp::socket& socket)
// {
//     boost::asio::streambuf buf;
//     boost::asio::read_until(socket, buf, "\n");
//     string data = boost::asio::buffer_cast<const char*>(buf.data());
//     return data;
// }

void send_(tcp::socket& socket, const string& message)
{
    const string msg = message + "\n";
    boost::asio::write(socket, boost::asio::buffer(message));
}

int main(int argc, char** argv) 
{
     boost::asio::io_service io_service;

     tcp::acceptor acceptor_(io_service, tcp::endpoint(tcp::v4(), 1234));
     //socket creation
     tcp::socket socket_(io_service);

     // Waiting for connection
     cout << "Waiting for connection" << endl;
     acceptor_.accept(socket_);

     int cnt = 1;

     while(true)
     {
          // send_(socket_, "Thank you DLR! N."+std::to_string(cnt));
          send_(socket_, "Thank you DLR! N.");
          
          cout << "Server sent message to Client" << endl;
          
          boost::this_thread::sleep_for(boost::chrono::seconds(1));
     }
     
     return 0;
}
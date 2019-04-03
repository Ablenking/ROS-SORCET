
#include <ros/ros.h>

#include  <sys/socket.h>      /* basic socket definitions */
//#include  <netinet/in.h>      /* sockaddr_in{} and other Internet defns */
#include  <arpa/inet.h>       /* inet(3) functions */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "TCPClient");
    ros::NodeHandle client_NodeHandle("~");

    int sock_fd,connect_fd=-3;
    std::string tcp_message = "Hi,TCP ,this is 127.0.0.1";

    struct sockaddr_in addr_serv;

    sock_fd = socket(AF_INET, SOCK_STREAM, 0);
     if(sock_fd == -1)
    {
        ROS_INFO("create client socket error");
       // SOCK_ERR("ERR:create client socket error");
        close(sock_fd);
        exit(1);
    }

    ROS_INFO("test handle 2");

   // addr_serv.addr_serv="127.0.0.1";
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(1314);//port
    addr_serv.sin_addr.s_addr = inet_addr("127.0.0.1");//server ip

    ros::Rate loop_rate(0.1);
    memset(&addr_serv.sin_zero, 0, sizeof(addr_serv.sin_zero));//memory apply ps:we can ignore it

    ROS_INFO("test handle 3");
    while(ros::ok())
    {
        if(connect_fd < 0)
            connect_fd = connect(sock_fd,(struct sockaddr *)&addr_serv ,sizeof(addr_serv) );
        if( connect_fd == -1) //try to connet if connet fail last time
            ROS_INFO("ROS Erro MSG:TCP connect err");
        else
        {
            ROS_INFO("TCP connet success!");
            write(sock_fd, tcp_message.c_str(), tcp_message.length()) ;
        }
        loop_rate.sleep();// try to connect sever
    }
    close(sock_fd);//close conneting
    return 0;
}

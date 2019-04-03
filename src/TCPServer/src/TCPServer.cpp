
#include <ros/ros.h>
#include <stdio.h>
#include  <sys/socket.h>      /* basic socket definitions */
//#include  <netinet/in.h>      /* sockaddr_in{} and other Internet defns */
#include  <arpa/inet.h>       /* inet(3) functions */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TCPServer");
    ros::NodeHandle client_NodeHandle("~");

    int socket_fd,accept_fd,read_fd_size;
    struct sockaddr_in addr_server;
    char recieve_data[100]={};
    
    memset( (void*)&addr_server, 0, sizeof(addr_server) );
    addr_server.sin_family = AF_INET;//IPV4 communication domain
    addr_server.sin_addr.s_addr = INADDR_ANY;//accept any address
    addr_server.sin_port = htons(1314);//change port to netchar

    socket_fd = socket(AF_INET, SOCK_STREAM, 0); // 1.creat socket
    if(socket_fd < 0)
    {
        ROS_INFO("SOCKET ERRO: create erro failure!");
        close(socket_fd);
    }
    ros::Rate loop_rate(4);
    while(ros::ok())
    {
        bind(socket_fd, (struct sockaddr *)&addr_server, sizeof(addr_server));// 2.bind the port and local ip
        listen(socket_fd,20);// 3.listen the client

        struct sockaddr_in addr_client;
        socklen_t len = sizeof(addr_client);

        accept_fd=accept(socket_fd, (struct sockaddr *)&addr_client, &len);// 4.wait for connet accept()
        if(accept_fd < -1)
        {
            ROS_INFO("accept erro");
            continue ;
        }
        write(accept_fd, "welcom", 7);
        read_fd_size = read(accept_fd, recieve_data, sizeof(recieve_data));  //5.
        printf("recieve data size : %d /n", read_fd_size);
        printf("I had recieve: %s /n",recieve_data);
        close(accept_fd);
        loop_rate.sleep();
    }
// 6.disconnet and close socket
   
  //  close(socket_fd);
    return 0;
}


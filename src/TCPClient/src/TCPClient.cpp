
#include <ros/ros.h>

#include  <sys/socket.h>      /* basic socket definitions */
//#include  <netinet/in.h>      /* sockaddr_in{} and other Internet defns */
#include  <arpa/inet.h>       /* inet(3) functions */


int main(int argc, char **argv)
{
    ros::init(argc, argv, "TCPClient");
    ros::NodeHandle client_NodeHandle("~");

    int sock_fd;
    char Data_[]="Hi,TCP ,this is 127.0.0.1";

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
                    ROS_INFO("test handle 3");
    while(ros::ok())
    {
        memset(&addr_serv.sin_zero, 0, sizeof(addr_serv.sin_zero));//memory apply
        if( -1 == connect(sock_fd,(struct sockaddr *)&addr_serv ,sizeof(struct sockaddr) )) //try to connet
        //    connect(sock_fd, (struct sockaddr*)&addr_serv, sizeof(struct sockaddr)) 
        {
            ROS_INFO("ROS Erro MSG:TCP connect err");
            ROS_INFO("close socket...");
            close(sock_fd);//close conneting
        }
        else{
            ROS_INFO("TCP connet success!");
            while(ros::ok)
            {
                if( 0 < send(sock_fd,Data_,sizeof(Data_),0) );//send data
                    ROS_INFO("data send success!");
                loop_rate.sleep();
            }
        }
        loop_rate.sleep();// try to connect sever
    }
    close(sock_fd);//close conneting
    return 0;
}

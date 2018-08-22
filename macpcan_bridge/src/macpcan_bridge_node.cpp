#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <can_msgs/Frame.h>

#define PCAN_CHANNEL PCAN_USBBUS1
#define PCAN_BAUDRATE PCAN_BAUD_500K

#ifndef __APPLE__
#include <asm/types.h>
#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
#include "PCANBasic.h"
#else
#include "PCBUSB.h"
#endif

void sigterm(int signo)
{
	/*printf("got signal %d\n", signo);*/
	CAN_Uninitialize(PCAN_NONEBUS);
	exit(0);
}
namespace macpcan_bridge
{
    class MacPCAN
    {
    public:
        MacPCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param);
    private:
        ros::Publisher can_pub_;
        ros::Subscriber can_sub_;

        void msgCallback(const can_msgs::Frame::ConstPtr& msg);

    };
    void convertPCANToMessage(const TPCANMsg* f, can_msgs::Frame& m)
    {
      m.id = static_cast<unsigned int>(f->ID);
      m.dlc = f->LEN;
      m.is_rtr = (f->MSGTYPE & PCAN_MESSAGE_RTR);
      m.is_extended = (f->MSGTYPE & PCAN_MESSAGE_EXTENDED) >> 1;
      m.is_error = (f->MSGTYPE & PCAN_MESSAGE_ESI) >> 4;


      for (unsigned long i = 0; i < 8; i++)  // always copy all data, regardless of dlc.
      {
        m.data[i] = f->DATA[i];
      }

    };
}// namespace macpcan_bridge
int main(int argc, char *argv[]) 
{
	TPCANMsg message;
	TPCANStatus status;
	int fd;
	fd_set fds;

	if((signal(SIGTERM, sigterm) == SIG_ERR) ||
	   (signal(SIGINT, sigterm) == SIG_ERR)) {
		perror("Error");
		return errno;
	}
	status = CAN_Initialize(PCAN_CHANNEL, PCAN_BAUDRATE, 0, 0, 0);
	printf("Initialize CAN: 0x%lx\n", status);
	if(status != PCAN_ERROR_OK) goto leave;

	status = CAN_GetValue(PCAN_CHANNEL, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
	if(status != PCAN_ERROR_OK) goto leave;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	while(select(fd+1, &fds, NULL, NULL, NULL) > 0) {
		status = CAN_Read(PCAN_CHANNEL, &message, NULL);
		if (status != PCAN_ERROR_OK) {
			printf("Error 0x%lx\n", status);
			break;
		}
		printf("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
				(int) message.ID, (int) message.LEN, (int) message.DATA[0],
				(int) message.DATA[1], (int) message.DATA[2],
				(int) message.DATA[3], (int) message.DATA[4],
				(int) message.DATA[5], (int) message.DATA[6],
				(int) message.DATA[7]);
	}
leave:
	CAN_Uninitialize(PCAN_NONEBUS);
	return 0;
}


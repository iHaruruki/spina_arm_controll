
//A0p-XXX(Xは総合角度)で全体の角度を変更
//CYp-XXX(Yはモジュール番号)でモジュール単体の角度を変更

/////////////////////////////////////////////////////////
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/wait.h>
#include <iostream>
using namespace std;
#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 2000000
#define BUFSIZE 100

int main(int argc, char *argv[]){

        unsigned char txData[BUFSIZE]={};
	unsigned char txData_r[BUFSIZE]={};
        int fd;

        struct termios oldtio, newtio;
        
        fd = open(SERIAL_PORT, O_RDWR);
        
        if(fd < 0){
                printf("open error");
        }
        cout << "fd:" << fd << endl;
		
	ioctl(fd, TCGETS, &oldtio);
	newtio.c_cflag = BAUDRATE | CS8 | CREAD;
	tcsetattr(fd, TCSANOW, &newtio);
	ioctl(fd, TCSETS, &newtio);
			
        printf("\n\rstart\n\r\n\r");
        
        while(1) {
        	char buf[9]; // バッファ
        	char send[11];         //コマンドを保存する配列
        	
		for(int i=0;i<8;i++)scanf("%c",&buf[i]);	
	    	printf("Received command:");
	    	for(int i=0;i<7;i++)printf("%c",buf[i]);
	    	printf("\n\r");
	    	
	    	int angle=((buf[4]-'0')*100) + ((buf[5]-'0')*10) + (buf[6]-'0');
	    	//if(buf[3]=='+')angle=angle;
	    	//else if(buf[3]=='-')angle=-angle;
	    	//else angle=1000;
	    	
	    	if(buf[0]=='C' && -30<=angle && angle<=30){
	    	
		    	send[0] = 0xAA;
			send[1] = 0xC6;
			send[2] = 0x00;
			send[3] = 0x00;
	    		send[4]='C';
    			send[5]=buf[1];
    			send[6]=buf[2];
    			send[7]=buf[3];
    			send[8]=(angle/10)+'0';
    			send[9]=angle-((angle/10)*10)+'0';
    			send[10] = 0x55;
    			
    			write(fd, send, sizeof(send));  //保存したコマンドを送信
    			usleep(1.0*50000);
    			
    			printf("send:");             //送信確認
			for(int i=4;i<10;i++)printf("%c",send[i]);
			printf(" length:%d\n\r",sizeof(send));
	    	}
	    	
	    	else if(buf[0]=='A' && -180<=angle && angle<=180){
	    		for(int i=0;i<6;i++){
	    			
	    			send[0] = 0xAA;
				send[1] = 0xC6;
				send[2] = 0x00;
				send[3] = 0x00;
	    			send[4]='C';
	    			send[5]=i+1+'0';
	    			send[6]=buf[2];
	    			send[7]=buf[3];
	    			send[8]=(angle/60)+'0';
	    			send[9]=(angle/6-((angle/60)*10))+'0';
	    			send[10] = 0x55;
	    			
	    			write(fd, send, sizeof(send));  //保存したコマンドを送信
				usleep(1.0*50000);
				printf("send:");             //送信確認
				for(int i=4;i<10;i++)printf("%c",send[i]);
				printf(" length:%d\n\r",sizeof(send));
    			}
	    	}
	    	
	    	else printf("Invalid value!\n\r");
	    	printf("\n\r");
        }
        ioctl(fd, TCSETS, &oldtio);
        close(fd);
}


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>

int main(int argc, char* argv[])
{
	char *map_addr;
	unsigned long phyrbc_addr = 0xf9c00000;
	int i;
	FILE *fp;
	int fd;
	unsigned int len;

        int map_fd = open("/dev/mem", O_RDWR);
        if(map_fd < 0)
        {
                printf("cannot open file /dev/mem\n");
                return 0;
        }
        
        map_addr = mmap(0, 24000000, PROT_READ|PROT_WRITE, MAP_SHARED, map_fd, phyrbc_addr);
	if(!map_addr)
		printf("map error\n");
        
	printf("rbc_data:\n");
	for(i=0;i<500;i++)
		printf("%x ",((unsigned short *)((unsigned int)map_addr))[i]);
	printf("\n");


	fp = fopen("./rbc", "wb");
	if(!fd)
		printf("fopen failed\n");

	len = fwrite(map_addr,sizeof(unsigned char),24000000,fp);	
	printf("fwitre len:%u\n",len);


        munmap(map_addr, phyrbc_addr);
        close(map_fd);
	fclose(fp);

        return 0;
        
}

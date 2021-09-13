#ifndef __INTERFACE_H
#define __INTERFACE_H


typedef struct
{
	unsigned char Directory_Num;
	unsigned char x;
	unsigned char y;
	char *p;
	unsigned short FC;
	unsigned short BC;
}directory;

void Show_Menu_One(directory *directory_DP,unsigned char Select_Frame);
void Show_Menu_All(directory *directory_DP,unsigned char num,unsigned char Select_Frame);
void Load_Animation(void);
void User_Main_InterFace(unsigned char show);
void SD_Casd_InterFace(unsigned char Show);
void User_Modules_InterFace(unsigned char show);
#endif	/*__INTERFACE_H*/

#ifndef DATASTORE_H
#define DATASTORE_H


#define  FLASHNUM  400


bool My_FlashWrite(uint16 Boot);
bool My_FlashRead(uint16 Boot);

void DataSave(void);
void DataLoad(void);
void Data_init(void);

#endif
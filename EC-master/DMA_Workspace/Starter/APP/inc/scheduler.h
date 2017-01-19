/*
 * scheduler.h
 *
 *  Created on: May 25, 2016
 *      Author: Chen Si
 */

#ifndef USER_SCHEDULER_H_
#define USER_SCHEDULER_H_

#define MAXTASKS 10
volatile unsigned char tasktimers[MAXTASKS];//array to store the time remaining for next executions of the tasks
#define FOREVER 255
#define _SS static unsigned char _lc=0; switch(_lc){default: // to be placed at the beginning of a task. Jump to the previously executed line before waiting using switch
#define _EE ;}; _lc=0; return 255;	// to be placed at the end of tasks.
#define WaitX(ticks)  do {_lc=(__LINE__+((__LINE__%256)==0))%256; return ticks ;} while(0); case (__LINE__+((__LINE__%256)==0))%256: //non-busy wait. Save current line number and perform other tasks while waiting for the duration
#define RunTask(TaskName,TaskID) do { if (tasktimers[TaskID]==0) tasktimers[TaskID]=TaskName(); }  while(0); // check if a task is due to executed, and run the task if that is the case.
#define RunTaskA(TaskName,TaskID) { if (tasktimers[TaskID]==0) {tasktimers[TaskID]=TaskName(); continue;} } // similar to the above, but jump to the start of the main loop. For prioritising different tasks.
#define InitTasks() {unsigned char i; for(i=MAXTASKS;i>0 ;i--) tasktimers[i-1]=0; }	//initialise task array
#define UpdateTimers() {unsigned char i; for(i=MAXTASKS;i>0 ;i--){if((tasktimers[i-1]!=0)&&(tasktimers[i-1]!=255)) tasktimers[i-1]--;}} //decrement waiting timer of all tasks at regular intervals
#define SEM uint8_t	// semaphore type
#define InitSem(sem) sem=0;	// initialise semaphore
#define WaitSem(sem) do{ sem=1; WaitX(0); if (sem>0) return 1;} while(0);	//non-busy wait for a semaphore
#define WaitSemX(sem,ticks)  do { sem=ticks+1; WaitX(0); if(sem>1){ sem--;  return 1;} } while(0);	//non-busy wait for a semaphore or timeout, whichever is earlier
#define WaitUntil(condition) do {_lc=(__LINE__+((__LINE__%256)==0))%256; }while(0); case (__LINE__+((__LINE__%256)==0))%256: if(!(condition)){return 0 ;} //non-busy wait until a condition is true
#define SendSem(sem)  do {sem=0;} while(0);	//sending a semaphore
#define CallSub(SubTaskName) do {unsigned char currdt; _lc=(__LINE__%255)+1; return 0; case (__LINE__%255)+1: currdt=SubTaskName(); if(currdt!=255) return currdt;} while(0);

#endif /* USER_SCHEDULER_H_ */

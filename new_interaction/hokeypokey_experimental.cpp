/*
This program makes the Darwin just do whatever the Kinect computer tells it to do
The Kinect tells the Darwin what body part to move and what stage of the experiment its in (to incorporate delays if needed)
The only thing this program tracks during the interaction is the previous action it performed
This is just in case the robot loses connection to the Kinect computer so that it can reset itself properly
*/

//THIS IS STANDING UP VERSION OF THE HOKEY POKEY FOR THE ROBOT

#include <stdio.h>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <vector>
#include "LinuxDARwIn.h"

#define SERVER_PORT htons(3333);

using namespace Robot;

void defaultposition();
void leftarmin();
void leftarmtiltout();
void leftarmtiltin();
void leftarmbacktodefault();
void rightarmin();
void rightarmtiltout();
void rightarmtiltin();
void rightarmbacktodefault();
void robotwait(double);
void leftlegin();
void leftleglifted();
void leftlegtiltin();
void leftlegtiltout();
void leftlegbacktodefault();
void rightlegin();
void rightleglifted();
void rightlegtiltin();
void rightlegtiltout();
void rightlegbacktodefault();
void handsup();
void handsleft();
void handsright();
void handsupbacktodefault();
void doHokeyPokey();
void lookup();
void lookdown();
void lookleft();
void lookright();
void spazout();
void limbIn(std::string);
void limbOut(std::string);
void limbShake(std::string);
void handleMistake(std::string);
std::string chooseUnderstoodMsg();
std::string chooseContinueMsg();
std::string chooseErrorMsg();

//motor 19 scan
//looking left: 
//looking right: 1800
//looking: 2280
//motor 20 pan
//looking down: 2100
//looking up: 2500

//Array values for the different positions for the robot

//to assist with balance when moving legs
int bentforwardArray[] = {2586,1603,1570,2563,1462,2487,1996,2007,2057,2056,1789,2311,2093,2003,2037,2013,2036,2055,1974,2050};

//
int leftarminArray[] = {2594,1212,1575,2563,1463,2487,1991,1997,2055,2061,1954,2108,2112,1981,2021,2032,2046,2058,1982,2055};

//
int leftarmtiltinArray[] = {2604,1212,1574,2596,1465,2489,1995,2007,2057,2061,1946,2107,2110,1981,2022,2020,2047,2060,1984,2050};

//lefthandtiltout
int leftarmtiltoutArray[] = {2604,1210,1572,2302,1464,2489,1996,2011,2057,2061,1947,2106,2109,1981,2024,2018,2047,2060,1984,2050};

//rightarmin
int rightarminArray[] = {2988,1577,1568,2561,1459,2477,1990,1999,2061,2051,1961,2105,2114,1979,2021,2035,2049,2047,1982,2293};

//rightarmtiltin
int rightarmtiltinArray[] = {2987,1581,1502,2573,1459,2477,1997,2005,2060,2056,1955,2102,2115,1979,2013,2034,2048,2049,1981,2293};

//rightarmtiltout
int rightarmtiltoutArray[] = {2987,1581,1894,2570,1459,2477,1995,2004,2060,2055,1955,2102,2115,1980,2013,2034,2047,2048,1982,2293};

int standingdefaultArray[] = {1936,2188,1592,2522,1391,2702,2014,1978,2090,2043,1869,2206,2291,1854,2113,2030,2068,2011,2018,2233};

int leftlegliftedArray[] = {1934,2112,1592,2525,1361,2853,2075,2062,2066,2030,2013,2646,2026,1454,1976,2120,2052,1995,2032,2313};

int leftleginArray[] = {1934,2112,1592,2525,1361,2852,2075,2009,2066,2009,2013,2701,2027,2074,1976,2224,2052,2046,2033,2313};

int leftlegtiltinArray[] = {1934,2112,1592,2518,1361,2853,2075,2248,2067,2024,2013,2716,2027,2073,1976,2232,2053,2041,2032,2313};

int leftlegtiltoutArray[] = {1934,2112,1592,2518,1361,2852,2075,1848,2067,1975,2013,2691,2027,2073,1976,2204,2052,2041,2032,2313};

int rightlegliftedArray[] = {1920,2112,1594,2524,1361,2852,2039,2054,2095,2025,1483,2066,2913,2048,2332,2078,2059,1998,2033,2312};

int rightleginArray[] = {1920,2112,1593,2524,1361,2853,2001,2054,2044,2025,1378,2067,2025,2048,1879,2078,2057,1998,2033,2313};

int rightlegtiltinArray[] = {1920,2112,1593,2524,1361,2853,1889,2055,2016,2026,1354,2066,2024,2048,1869,2078,2078,1998,2033,2313};

int rightlegtiltoutArray[] = {1920,2112,1594,2524,2361,2852,2244,2055,2083,2026,1374,2067,2025,2048,1790,2078,2079,1998,2033,2312};

int handsupArray[] = {3678,313,1563,2479,1697,2450,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

int handsleftArray[] = {-1,-1,1712,2358,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

int handsrightArray[] = {-1,-1,1439,2479,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

//std::string understoodResponses[] = {"understood.mp3", "okay.mp3", "alright.mp3"};
std::string understoodResponses[] = {"understood.mp3", "okay.mp3", "alright.mp3", "hmmm.mp3", "okay_indecisive.mp3", "umm.mp3"}; 
/*Combined the understoodResponses and errorResponses because if the robot only says umm when it makes a mistake
Then the robot knows or has an idea when its making a mistake, therefore defeating the purpose of the whole experiment
At least with this approach, I can use the "varying degress of confience" narrative with participants*/
std::string continueResponses[] = {"continue.mp3", "please_continue.mp3", "next_step.mp3"};
std::string errorResponses[] = {"hmmm.mp3", "okay_indecisive.mp3", "umm.mp3"};
std::string chosen = "";

bool skipContinueMessage = false;

std::vector<std::string> stringSplit(std::string);

std::string delimeter = " "; //space delimeter

//Initialize framework
LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);

clock_t goal;

int tmp, motorid;

int theta = 10;

std::string previousState = "";

int main()
{
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
    
    int value;
    std::srand(time(NULL));
    
	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		/*cm730.WriteWord(id, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteByte(id, MX28::P_P_GAIN, 5, 0);
		cm730.WriteByte(id, MX28::P_LED, 1, 0);*/
	
		if(cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, 0) != CM730::SUCCESS)
		{
			printf("Problem. Terminating program.\n");
			return 0;
		}
	}
	
	usleep(2000);

	defaultposition();

	//the Darwin will announce when the program is actually running
	LinuxActionScript::PlayMP3("programStart.mp3");
	
	char receivedStr[1000];
	for(int i=0; i<1000; i++)
	{
		receivedStr[i] = '\0';
	}
	
	//defining the socket properties
	int serverSock = socket(AF_INET, SOCK_STREAM, 0);
	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = SERVER_PORT;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	
	//keep track of the previous state locally just in case the robot loses connection
	//all of the logic, including another copy of previous state, is handled on the Kinect computer for the interaction
	//std::string previousState = "";
	/* Possible values for previousState
	default - robot just standing 
	getStarted - when participant notifies that he/she is ready to start, same as default
	rightArmIn/leftArmIn - robot sticking out its arm
	rightLegIn/leftLegIn - robot sticking out its leg
	rightarmin/leftarmin - same as default 
	rightArmShake/leftArmShake - robot shakes out arm horizontally (left to right)
	rightLegShake/leftLegShake - robot shakes  leg vertically (up down)
	hokeyPokey - robot raises both arms up and shakes them back and forth*/

	//the message the Darwin sends back to the Kinect
	std::string messageToKinect = "";
	
	while(true)
	{
		bind(serverSock, (struct sockaddr*)&serverAddr, sizeof(struct sockaddr));
		listen(serverSock,4);
		sockaddr_in clientAddr;
		socklen_t sin_size = sizeof(struct sockaddr_in);
        
		int clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &sin_size);
		
		LinuxActionScript::PlayMP3("connected.mp3");
		robotwait(2.0);
        
		//std::string action, inputDelay, yesOrNoMovement;
		std::string limb;
		//int movementNumber;
		int status, index;
		//bool mistakeFlag;
		
		do{
			status = recv(clientSock, receivedStr, 1000, 0);
			
			if(status > 0)
			{
				index = 0;
				while(receivedStr[index] != '\0')
				{
					index++;
				}
				
				std::string holder(receivedStr, receivedStr + index);

				std::cout << "Received: " << holder << std::endl;
				//holder consists of 3 words
				//first word will tell the robot what action to perform
				//second word will tell the robot what state it is in (no delay, small delay, large delay)
				//third word will tell the robot if the action to perform is correct or not (for stage 2 where correct actions may have shorter delays than incorrect ones)
				
				//Update: holder would consist of a message that is a multiple of 2
				//ex: leftArm 0 rightArm 3
				//in the above, there are no mistakes for the left arm, but the right arm makes a mistake on the third move for the right arm (the second rightArmIn)

				std::vector<std::string> tokens;
				tokens = stringSplit(holder);

				if(tokens.size() == 1) //then the message is either: repeated, wrongconfirmation, rightconfirmation, newlimb, getStarted, or actionPerformed
				{
					if(tokens[0] == "repeated")
					{
						LinuxActionScript::PlayMP3("repeatedMovement.mp3");
						robotwait(4.0);
						messageToKinect = "Robot played repeatedMovement.mp3 to signify that this was a repeated movement.";
					}
					else if(tokens[0] == "humanError")
					{
						LinuxActionScript::PlayMP3("humanErrorDifferentLimb.mp3");
						robotwait(4.0);
						messageToKinect = "Robot played humanErrorDifferentLimb.mp3 to signify that the participant presented a limb other than the current limb.";
					}
					else if(tokens[0] == "wrongconfirmation")
					{
						LinuxActionScript::PlayMP3("incorrectLimbQuery.mp3");
						robotwait(6.0);
						messageToKinect = "Robot played incorrectLimbQuery.mp3 to signify that it made a mistake during the attempted dance and wants to see the demonstration again.";
					}
					else if(tokens[0] == "rightconfirmation")
					{
						LinuxActionScript::PlayMP3("nextlimbQuery.mp3");
						robotwait(6.0);
						messageToKinect = "Robot played nextlimbQuery.mp3 to signify that it did the dance correctly and wants to learn the next limb.";
					}
					else if(tokens]0] == "newlimb")
					{
						LinuxActionScript::PlayMP3("readyNewLimb.mp3");
						robotwait(3.0);
						messageToKinect = "Robot played readyNewLimb.mp3 to signify that it is ready to learn the next limb.";
					}
					else if(tokens[0] == "repeatLimb")
					{
						LinuxActionScript::PlayMP3("demonstrateAgain.mp3");
						robotwait(4.0);
						messageToKinect = "Robot played demonstrateAgain.mp3 to signify that it wants to see the demonstration of the limb again.";
					}
					else if(tokens[0] == "getStarted")
					{
						LinuxActionScript::PlayMP3("ready_to_begin.mp3");
						robotwait(9.0);
						messageToKinect = "Robot played ready_to_begin.mp3 to signify the start of the interaction.";
					}
					else if(tokens[0] == "actionPerformed")
					{
						chosen = chooseContinueMsg();
						LinuxActionScript::PlayMP3(chosen.c_str());
						robotwait(2.0);
						messageToKinect = "Robot played " + chosen + " to signify that it saw the participant's movement.";
					}
				}
				else //then it is a sequence of movements
				{
					LinuxActionScript::PlayMP3("attemptingDance.mp3");
					robotwait(10.0);
					messageToKinect = "";
					for(int r=0; r<tokens.size(); r++)
					{
						if(tokens[r] == "leftArmIn")
						{
							if(!(previousState == "leftArmIn" || previousState == "leftArmShake")) //if the previous state is neither one of these
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
								}
							}
							
							//LinuxActionScript::PlayMP3("lefthandin.mp3");

							leftarmin();
							robotwait(1.0);

							previousState = "leftArmIn";
							messageToKinect += "leftArmIn ";
						}
						else if(tokens[r] == "leftArmShake")
						{
							if(!(previousState == "leftArmIn" || previousState == "leftArmShake"))
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
									leftarmin();
									robotwait(1.0);
								}
							}
							
							//LinuxActionScript::PlayMP3("lefthandshake.mp3");

							leftarmtiltout();
							robotwait(1.0);
							leftarmtiltin();
							robotwait(1.0);
							leftarmtiltout();
							robotwait(1.0);
							leftarmtiltin();
							robotwait(1.0);
							leftarmtiltout();
							robotwait(1.0);

							previousState = "leftArmShake";
							messageToKinect += "leftArmShake ";
						}
						else if(tokens[r] == "rightArmIn")
						{
							if(!(previousState == "rightArmIn" || previousState == "rightArmShake"))
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
								}
							}
							
							//LinuxActionScript::PlayMP3("righthandin.mp3");

							rightarmin();
							robotwait(1.0);

							previousState = "rightArmIn";
							messageToKinect += "rightArmIn ";
						}
						else if(tokens[r] == "rightArmShake")
						{
							if(!(previousState == "rightArmIn" || previousState == "rightArmShake"))
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
									rightarmin();
									robotwait(1.0);
								}
							}
							
							//LinuxActionScript::PlayMP3("righthandshake.mp3");

							rightarmtiltout();
							robotwait(1.0);
							rightarmtiltin();
							robotwait(1.0);
							rightarmtiltout();
							robotwait(1.0);
							rightarmtiltin();
							robotwait(1.0);
							rightarmtiltout();
							robotwait(1.0);

							previousState = "rightArmShake";
							messageToKinect += "rightArmShake ";
						}
						else if(tokens[r] == "leftLegIn")
						{
							if(!(previousState == "leftLegIn" || previousState == "leftLegShake"))
							{
								//check legs
								if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
								}
							}

							//LinuxActionScript::PlayMP3("leftfootin.mp3");

							leftleglifted();
							robotwait(0.5);
							leftlegin();
							robotwait(1.0);

							previousState = "leftLegIn";
							messageToKinect += "leftLegIn ";
						}
						else if(tokens[r] == "leftLegshake")
						{
							if(!(previousState == "leftLegIn" || previousState == "leftLegShake"))
							{
								if(previousState == "rightLegIn" || previousState == "rightLegShake") 
								{
									rightlegbacktodefault();
									robotwait(1.0);
									leftleglifted();
									robotwait(0.5);
									leftlegin();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
									leftleglifted();
									robotwait(0.5);
									leftlegin();
									robotwait(1.0);
								}
							}

							//LinuxActionScript::PlayMP3("leftfootshake.mp3");

							leftlegtiltout();
							robotwait(1.0);
							leftlegtiltin();
							robotwait(1.0);
							leftlegtiltout();
							robotwait(1.0);
							leftlegtiltin();
							robotwait(1.0);
							leftlegin();
							robotwait(1.0);
							leftleglifted();

							previousState = "leftLegShake";
							messageToKinect += "leftLegShake ";
						}
						else if(tokens[r] == "rightLegIn")
						{
							if(!(previousState == "rightLegIn" || previousState == "rightLegShake"))
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
								}
							}

							//LinuxActionScript::PlayMP3("rightfootin.mp3");

							rightleglifted();
							robotwait(0.5);
							rightlegin();
							robotwait(1.0);

							previousState = "rightLegIn";
							messageToKinect += "rightLegIn ";
						}
						else if(tokens[r] == "rightLegShake")
						{
							if(!(previousState == "rightLegIn" || previousState == "rightLegShake"))
							{
								//check legs
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(1.0);
									leftleglifted();
									robotwait(0.5);
									leftlegin();
									robotwait(1.0);
								}
								else
								{
									defaultposition();
									robotwait(1.0);
									rightleglifted();
									robotwait(0.5);
									rightlegin();
									robotwait(1.0);
								}
							}

							//LinuxActionScript::PlayMP3("rightfootshake.mp3");

							rightlegtiltout();
							robotwait(1.0);
							rightlegtiltin();
							robotwait(1.0);
							rightlegtiltout();
							robotwait(1.0);
							rightlegtiltin();
							robotwait(1.0);
							rightlegin();
							robotwait(1.0);
							rightleglifted();

							previousState = "rightLegShake";
							messageToKinect += "rightLegShake ";
						}
						else if(tokens[r] == "hokeyPokey")
						{
							if(previousState != "hokeyPokey")
							{
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									//lower the left leg 
									//this is different from telling the Darwin to go to default since you have to carefully adjust the knee, hip, and ankle motors
									leftlegbacktodefault();
									robotwait(1.0);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									//lower the right leg
									rightlegbacktodefault();
									robotwait(1.0);
								}
								/*else
								{
									defaultposition(); //the arms can go back to the default position no problem
									robotwait(1.0);
								}*/
							}

							//LinuxActionScript::PlayMP3("hokeypokey.mp3");
							
							//do the hokey pokey
							doHokeyPokey();
							robotwait(1.0);
							previousState == "hokeyPokey";
							messageToKinect += "hokeyPokey ";
						}
						else //going to assume it's default
						{
							if(previousState != "default")
							{
								if(previousState == "leftLegIn" || previousState == "leftLegShake")
								{
									leftlegbacktodefault();
									robotwait(0.5);
								}
								else if(previousState == "rightLegIn" || previousState == "rightLegShake")
								{
									rightlegbacktodefault();
									robotwait(0.5);
								}
							}

							defaultposition();
							robotwait(1.0);
							previousState = "default";
							messageToKinect += "default ";
						}
						robotwait(0.5); //a little break between each movement
					} //end for loop
				}
				
				//now the robot needs to ask if it did the right thing
				LinuxActionScript::PlayMP3("correctnessQuery.mp3");
				robotwait(3.0);
				messageToKinect += ". Robot played correctnessQuery.mp3 to ask if it performed the dance correctly.";

				//respond back to client saying it is ready for next message
				send(clientSock, messageToKinect.c_str(), messageToKinect.length(), 0);
			}
		} while(status > 0);	
		
		//audible notification if the connection is lost
		LinuxActionScript::PlayMP3("lostConnection.mp3");
		
		if(previousState == "rightLegIn" || previousState == "rightLegShake")
		{
			rightlegbacktodefault();
			robotwait(1.0);
		}
		else if(previousState == "leftLegIn" || previousState == "leftLegShake")
		{
			leftlegbacktodefault();
			robotwait(1.0);
		}
		else if(previousState == "rightArmIn" || previousState == "rightArmShake")
		{
			rightarmbacktodefault();
			robotwait(1.0);
		}
		else if(previousState == "leftArmIn" || previousState == "leftArmShake")
		{
			leftarmbacktodefault();
			robotwait(1.0);
		}
	} 
	return 0;
}

std::string chooseUnderstoodMsg()
{
	//int i = std::rand() % 3; //pick random number between 0 and 2
	int i = std::rand() % 6; //pick random number between 0 and 5
	return understoodResponses[i];
}

std::string chooseContinueMsg()
{
	int i = std::rand() % 3; //pick random number between 0 and 2
	return continueResponses[i];
}

std::string chooseErrorMsg()
{
	int i = std::rand() % 3; //pick random number between 0 and 2
	return errorResponses[i];
}

void robotwait(double lengthInSeconds)
{
    goal = (lengthInSeconds * CLOCKS_PER_SEC) + clock();
    while(goal > clock())
    {
    }
}

//to parse input from the Kinect computer
std::vector<std::string> stringSplit(std::string temp)
{
	std::vector<std::string> buffer;
	
	size_t pos = 0;
	std::string token;
	while((pos = temp.find(delimeter)) != std::string::npos)
	{
		token = temp.substr(0, pos);
		buffer.push_back(token);
		temp.erase(0, pos + delimeter.length());
	}

	//now we have the first word, now split up the last word which is the combined 2nd and 3rd word
	while((pos = temp.find(delimeter)) != std::string::npos)
	{
		token = temp.substr(0, pos);
		buffer.push_back(token);
		temp.erase(0, pos + delimeter.length());

	}

	buffer.push_back(temp);

	
	return buffer;
}

void defaultposition()
{
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
        cm730.WriteWord(id, MX28::P_GOAL_POSITION_L, standingdefaultArray[id-1], 0);
    }
}

/*----------------------------------------------
ARM PROGRAMMING
----------------------------------------------*/
void leftarmin()
{	
    //only care about servos 2, 4, and 6

    //motor 2
    motorid = 2;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarminArray[motorid-1])
    {
		for(int i=tmp;i>leftarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);
    
    //motor 4
    motorid = 4;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarminArray[motorid-1])
    {
		for(int i=tmp;i>leftarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);

    //motor 6
    motorid = 6;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarminArray[motorid-1])
    {
		for(int i=tmp;i>leftarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);
}

void leftarmtiltout()
{
    //only care about servos 2, 4, and 6

    //motor 2
    motorid = 2;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);

    //motor 4
    motorid = 4;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);

    //motor 6
    motorid = 6;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);
}

void leftarmtiltin()
{
    //only care about servos 2, 4, and 6
    
    //motor 2
    motorid = 2;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0); 

    //motor 4
    motorid = 4;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0); 

    //motor 6
    motorid = 6;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>leftarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0); 
}

void leftarmbacktodefault()
{
	//only care about servos 2, 4, and 6
    
    //motor 2
    motorid = 2;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0); 
	
	//motor 4
    motorid = 4;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 6
    motorid = 6;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

void rightarmin()
{
    //only care about servos 1, 3, and 5

    //motor 1
   	motorid = 1;
   	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarminArray[motorid-1])
    {
		for(int i=tmp;i>rightarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0); 

   	//motor 3
	motorid = 3;  
   	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarminArray[motorid-1])
    {
		for(int i=tmp;i>rightarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}   
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0); 

   	//motor 5
   	motorid = 5; 
   	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarminArray[motorid-1])
    {
		for(int i=tmp;i>rightarminArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarminArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0); 
}

void rightarmtiltout()
{
    //only care about servos 1, 3, and 5

    //motor 1
	motorid = 1; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0); 

    //motor 3
	motorid = 3; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0);

    //motor 5
	motorid = 5; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0);
}

void rightarmtiltin()
{
    //only care about servos 1, 3, and 5

    //motor 1
	motorid = 1;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0); 

    //motor 3
	motorid = 3;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0);

    //motor 5
	motorid = 5;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightarmtiltinArray[motorid-1])
    {
		for(int i=tmp;i>rightarmtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightarmtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0);
}

void rightarmbacktodefault()
{
	//only care about servos 1, 3, and 5
    
    //motor 1
    motorid = 1; 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
	
	//motor 3
    motorid = 3;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 5
    motorid = 5;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

/*----------------------------------------------
LEG PROGRAMMING
----------------------------------------------*/
void leftleglifted()
{
	//lock the all of the motors for the right leg first for support
	for(int i=7; i<=17; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//only care about motors 12, 14, and 16

	//motor 12
	motorid = 12; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>leftlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0);
	
    //motor 14
	motorid = 14;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>leftlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0);

    //motor 16
	motorid = 16;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>leftlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0);
}

void leftlegin()
{
	//lock the all of the motors for the right leg first for support
	for(int i=7; i<=17; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//only care about motors 12, 14, and 16

	//motor 12
	motorid = 12; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftleginArray[motorid-1])
    {
		for(int i=tmp;i>leftleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftleginArray[motorid-1], 0);
	
    //motor 14
	motorid = 14;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftleginArray[motorid-1])
    {
		for(int i=tmp;i>leftleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftleginArray[motorid-1], 0);

    //motor 16
	motorid = 16;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftleginArray[motorid-1])
    {
		for(int i=tmp;i>leftleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftleginArray[motorid-1], 0);
}

void leftlegtiltin()
{
	//lock the motors of the right leg and also all of the motors of the left leg that are not 8
	//right leg motors lock
	for(int i=7; i<=17; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//right leg motors lock
	for(int i=10; i<=18; i+=2)
	{
		 cm730.ReadWord(i, MX28::P_PRESENT_POSITION_L, &tmp, 0);
		 cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, tmp, 0);
	}

	motorid = 8;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftlegtiltinArray[motorid-1])
    {
		for(int i=tmp;i>leftlegtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftlegtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void leftlegtiltout()
{
	//lock the motors of the right leg and also all of the motors of the left leg that are not 8
	//right leg motors lock
	for(int i=7; i<=17; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//right leg motors lock
	for(int i=10; i<=18; i+=2)
	{
		 cm730.ReadWord(i, MX28::P_PRESENT_POSITION_L, &tmp, 0);
		 cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, tmp, 0);
	}

	motorid = 8;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftlegtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>leftlegtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftlegtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegtiltoutArray[motorid-1], 0);
}

void leftlegbacktodefault()
{
	//only care about motors 8, 10, 12, 14, and 16

	//motor 8
	motorid = 8;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0); 

	//motor 10
	motorid = 10;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 16
	motorid = 16;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
	
	robotwait(0.5); //delay to make sure the ankle goes back to the proper position before the rest of the leg moves

	leftleglifted();

	//motor 12
	motorid = 12;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

    //motor 14
    motorid = 14;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

void rightleglifted()
{
	//lock all of the motors for the left leg first for support
	for(int i=8; i<=18; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//only care about motors 11, 13, and 15

	//motor 11
	motorid = 11; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>rightlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0); 

	//motor 13
	motorid = 13;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>rightlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0);

    //motor 15
	motorid = 15;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightlegliftedArray[motorid-1])
    {
		for(int i=tmp;i>rightlegliftedArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightlegliftedArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0);
}

void rightlegin()
{
	//lock all of the motors for the left leg first for support
	for(int i=8; i<=18; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//only care about motors 11, 13, and 15

	//motor 11
	motorid = 11; 
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightleginArray[motorid-1])
    {
		for(int i=tmp;i>rightleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightleginArray[motorid-1], 0); 

	//motor 13
	motorid = 13;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightleginArray[motorid-1])
    {
		for(int i=tmp;i>rightleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightleginArray[motorid-1], 0); 

    //motor 15
	motorid = 15;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightleginArray[motorid-1])
    {
		for(int i=tmp;i>rightleginArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightleginArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}    
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightleginArray[motorid-1], 0);  
}

void rightlegtiltin()
{
	//lock the motors of the left leg and also all of the motors of the left leg that are not 7
	//right leg motors lock
	for(int i=8; i<=18; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//left leg motors lock
	for(int i=9; i<=17; i+=2)
	{
		 cm730.ReadWord(i, MX28::P_PRESENT_POSITION_L, &tmp, 0);
		 cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, tmp, 0);
	}

	motorid = 7;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightlegtiltinArray[motorid-1])
    {
		for(int i=tmp;i>rightlegtiltinArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightlegtiltinArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegtiltinArray[motorid-1], 0); 
}

void rightlegtiltout()
{
	//lock the motors of the left leg and also all of the motors of the left leg that are not 7
	//right leg motors lock
	for(int i=8; i<=18; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, standingdefaultArray[i-1], 0);
	}

	//left leg motors lock
	for(int i=9; i<=17; i+=2)
	{
		 cm730.ReadWord(i, MX28::P_PRESENT_POSITION_L, &tmp, 0);
		 cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, tmp, 0);
	}

	motorid = 7;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightlegtiltoutArray[motorid-1])
    {
		for(int i=tmp;i>rightlegtiltoutArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightlegtiltoutArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegtiltoutArray[motorid-1], 0);
}

void rightlegbacktodefault()
{
	//only care about motors 7, 9, 11, 13, and 15

	//motor 7
	motorid = 7;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 9
	motorid = 9;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 15
	motorid = 15;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
	
	robotwait(0.5); //delay to make sure the ankle goes back to the proper position before the rest of the leg moves
	rightleglifted();

	//motor 11
	motorid = 11;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

    //motor 13
	motorid = 13;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

//HOKEY POKEY HANDS UP CODE
void handsup()
{
	//only care about motors 1-6
	//go by arms at a time though

	//motor 1
	motorid = 1;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0); 

	//motor 3
	motorid = 3;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);

	//motor 5
	motorid = 5;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);

	//motor 2
	motorid = 2; 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);

	//motor 4
	motorid = 4;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);

	//motor 6
	motorid = 6;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsupArray[motorid-1])
    {
		for(int i=tmp;i>handsupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);
}

void handsleft()
{
	//only care about motors 3 and 4 (left and right shoulders)

	//motor 3
	motorid = 3;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsleftArray[motorid-1])
    {
		for(int i=tmp;i>handsleftArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsleftArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsleftArray[motorid-1], 0);

	//motor 4
	motorid = 4;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsleftArray[motorid-1])
    {
		for(int i=tmp;i>handsleftArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsleftArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsleftArray[motorid-1], 0);
}

void handsright()
{
	//only care about motors 3 and 4
	
	//motor 3
	motorid = 3;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsrightArray[motorid-1])
    {
		for(int i=tmp;i>handsrightArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsrightArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsrightArray[motorid-1], 0);
	
	//motor 4
	motorid = 4;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= handsrightArray[motorid-1])
    {
		for(int i=tmp;i>handsrightArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<handsrightArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsrightArray[motorid-1], 0);
}

void handsupbacktodefault()
{
	//only care about motors 1-6

	//motor 1
	motorid = 1; 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0); 

	//motor 3
	motorid = 3; 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 5
	motorid = 5; 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 2
	motorid = 2;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 4
	motorid = 4;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);

	//motor 6
	motorid = 6;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= standingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>standingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<standingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

void doHokeyPokey()
{
	handsup();
	robotwait(1.0);
	handsleft();
	robotwait(1.0);
	handsright();
	robotwait(1.0);
	handsleft();
	robotwait(1.0);
	handsright();
	robotwait(1.0);
	handsupbacktodefault();
}

//motor 19 scan
//looking left: 
//looking right: 1800
//looking: 2280
//motor 20 pan
//looking down: 2100
//looking up: 2500

void lookup()
{
	motorid = 20;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
	for(int i=tmp; i<standingdefaultArray[motorid-1]; i=i+theta)
	{
		cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
		usleep(1);
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, standingdefaultArray[motorid-1], 0);
}

void lookdown()
{
	motorid = 20;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
	for(int i=tmp; i>2100; i=i-theta)
	{
		cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
		usleep(1);
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, 2100, 0);
}

void lookleft()
{
	motorid = 19;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
	if(tmp >= 2200)
	{
		for(int i=tmp; i>2200; i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp; i<2200; i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, 2200, 0);
}

void lookright()
{
	motorid = 19;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
	if(tmp >= 1800)
	{
		for(int i=tmp; i>1800; i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp; i<1800; i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			usleep(1);
		}
	}
	cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, 1800, 0);
}

void chooseErrorMovement(int temp)
{
	if(temp == 0)
		lookup();
	else if(temp == 1)
		lookdown();
	else if(temp == 2)
		lookleft();
	else if(temp == 3)
		lookright();
}

void spazout()
{
	int tempArray[] = {0,0,0,0};
	int count = 0;

	while(count < 4)
	{
		int i = std::rand() % 4; //pick random number between 0 and 2
		if(tempArray[i] == 0)
		{
			chooseErrorMovement(i);
			tempArray[i] = 1;
			count++;
		}
	}

	defaultposition();
}

void limbIn(std::string tempLimb)
{
	if(tempLimb == "leftArm")
	{
		leftarmin();
		robotwait(1.0);
		previousState == "leftArmIn";
	}
	else if(tempLimb == "rightArm")
	{
		rightarmin();
		robotwait(1.0);
		previousState == "rightArmIn";
	}
	else if(tempLimb == "leftLeg")
	{
		leftleglifted();
		robotwait(0.5);
		leftlegin();
		robotwait(1.0);
		previousState == "leftLegIn";
	}
	else if(tempLimb == "rightLeg")
	{
		rightleglifted();
		robotwait(0.5);
		rightlegin();
		robotwait(1.0);
		previousState == "rightLegIn";
	}
}

void limbOut(std::string tempLimb)
{
	if(tempLimb == "leftArm")
	{
		defaultposition();
		robotwait(1.0);
		previousState = "leftArmOut";
	}
	else if(tempLimb == "rightArm")
	{
		defaultposition();
		robotwait(1.0);
		previousState = "rightArmOut";
	}
	else if(tempLimb == "leftLeg")
	{
		if(previousState == "leftLegIn" || previousState == "leftLegShake")
		{
			leftlegbacktodefault();
			robotwait(0.5);
		}
		else if(previousState == "rightLegIn" || previousState == "rightLegShake")
		{
			rightlegbacktodefault();
			robotwait(0.5);
		}

		defaultposition();
		robotwait(1.0);
		previousState == "leftLegOut";
	}
	else if(tempLimb == "rightLeg")
	{ 
		if(previousState == "leftLegIn" || previousState == "leftLegShake")
		{
			leftlegbacktodefault();
			robotwait(0.5);
		}
		else if(previousState == "rightLegIn" || previousState == "rightLegShake")
		{
			rightlegbacktodefault();
			robotwait(0.5);
		}

		defaultposition();
		robotwait(1.0);
		previousState == "rightLegOut";
	}
}

void limbShake(std::string tempLimb)
{
	if(tempLimb == "leftArm")
	{
		leftarmtiltout();
		robotwait(1.0);
		leftarmtiltin();
		robotwait(1.0);
		leftarmtiltout();
		robotwait(1.0);
		leftarmtiltin();
		robotwait(1.0);
		previousState = "leftArmShake";
	}
	else if(tempLimb == "rightArm")
	{
		rightarmtiltout();
		robotwait(1.0);
		rightarmtiltin();
		robotwait(1.0);
		rightarmtiltout();
		robotwait(1.0);
		rightarmtiltin();
		robotwait(1.0);
		previousState = "leftArmShake";
	}
	else if(tempLimb == "leftLeg")
	{
		leftlegtiltout();
		robotwait(1.0);
		leftlegtiltin();
		robotwait(1.0);
		leftlegtiltout();
		robotwait(1.0);
		leftlegtiltin();
		robotwait(1.0);
		leftlegin();
		robotwait(1.0);
		leftleglifted();
		robotwait(1.0);
		previousState = "leftLegShake";
	}
	else if(tempLimb == "rightLeg")
	{
		rightlegtiltout();
		robotwait(1.0);
		rightlegtiltin();
		robotwait(1.0);
		rightlegtiltout();
		robotwait(1.0);
		rightlegtiltin();
		robotwait(1.0);
		rightlegin();
		robotwait(1.0);
		rightleglifted();
		robotwait(1.0);
		previousState = "leftLegShake";
	}
}

void handleMistake(std::string tempLimb)
{
	//TODO: A lot of stuff here

	//at the end, return to the default position
	if(tempLimb == "leftArm")
	{
		leftarmbacktodefault();
		robotwait(1.0);
	}
	else if(tempLimb == "rightArm")
	{
		rightarmbacktodefault();
		robotwait(1.0);
	}
	else if(tempLimb == "leftLeg")
	{
		leftlegbacktodefault();
		robotwait(1.0);
	}
	else if(tempLimb == "rightLeg")
	{
		rightlegbacktodefault();
		robotwait(1.0);
	}
}
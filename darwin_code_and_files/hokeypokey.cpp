/*
This program makes the Darwin just do whatever the Kinect computer tells it to do
The Kinect tells the Darwin what body part to move and what stage of the experiment its in (to incorporate delays if needed)
The only thing this program tracks during the interaction is the previous action it performed
This is just in case the robot loses connection to the Kinect computer so that it can reset itself properly
*/

//THIS IS SITTING DOWN VERSION OF THE HOKEY POKEY FOR THE ROBOT

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
void leftankleup();
void leftankledown();
void leftlegbacktodefault();
void rightlegin();
void rightankleup();
void rightankledown();
void rightlegbacktodefault();
void handsup();
void handsleft();
void handsright();
void handsupbacktodefault();
void bendforward();
void bendforwardbacktodefault();
void doHokeyPokey();
void lookup();
void lookdown();
void lookleft();
void lookright();
std::string chooseUnderstoodMsg();
std::string chooseContinueMsg();

//motor 19 scan
//looking left: 
//looking right: 1800
//looking: 2280
//motor 20 pan
//looking down: 2100
//looking up (Also default): 2500

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

int leftlegliftedArray[] = {2014,2094,1619,2347,1826,2356,2100,2039,2053,2031,1291,2973,2184,2081,2161,1719,2031,2057,2045,2526}; //only motors 12 and 14 should differ
//leftlegliftedArray[11] = 2458;
//leftlegliftedArray[13] = 1690;

int leftankledownArray[] = {2001,2125,1651,2353,1765,2218,2142,1941,2054,2031,1209,2941,2358,2063,2339,2292,1995,2131,1717,2606};

int leftankleupArray[] = {2002,2125,1651,2352,1765,2218,2142,1940,2054,2031,1211,2940,2358,2063,2342,1506,1995,2132,1715,2607};

//leftlegin
//int leftleginArray[] = {2587,1598,1574,2569,1461,2486,1997,2004,2061,2053,1936,2591,2092,1808,2038,2012,2033,2061,1975,2049};

//rightleglifted
int rightlegliftedArray[] = {2009,2140,1672,2346,1805,2342,2094,1925,2056,2031,1023,2871,2112,1761,2087,1737,2112,1956,1983};

int rightankledownArray[] = {2022,2140,1628,2346,1805,2342,2103,1921,2054,2032,1029,2869,2116,1762,1708,1736,2091,1956,1964,2615};

int rightankleupArray[] = {2012,2140,1628,2346,1805,2342,2104,1923,2056,2032,1028,2871,2116,1762,2493,1741,2091,1956,1980,2517};

//rightlegin
//int rightleginArray[] = {2618,1601,1564,2572,1415,2491,2005,1997,2082,2079,1623,2106,2009,1975,2032,2034,2035,2057,1984,2060};

//standingdefault
int sittingdefaultArray[] = {2000,2169,1674,2465,1711,2270,2130,1930,2053,2031,1232,2857,2311,1796,2376,1692,1997,2076,2054,2509};

//int bentforwardArray[] = {2596,1595,1577,2556,1458,2493,2002,2001,2049,2066,1789,2311,2110,1981,2023,2032,2042,2064,1977,2049};

int handsupArray[] = {3678,313,1563,2479,1697,2450,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

int handsleftArray[] = {-1,-1,1712,2358,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

int handsrightArray[] = {-1,-1,1439,2479,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

std::string understoodResponses[] = {"understood.mp3", "okay.mp3", "alright.mp3"};
std::string continueResponses[] = {"continue.mp3", "please_continue.mp3", "next_step.mp3"};
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
	std::string previousState = "";
	/* Possible values for previousState
	default - robot just standing 
	getStarted - when participant notifies that he/she is ready to start, same as default
	rightArmIn/leftArmIn - robot sticking out its arm
	rightLegIn/leftLegIn - robot sticking out its leg
	rightarmin/leftarmin - same as default 
	rightArmShake/leftArmShake - robot shakes out arm horizontally (left to right)
	rightLegShake/leftLegShake - robot shakes  leg vertically (up down)
	hokeyPokey - robot raises both arms up and shakes them back and forth*/

	//the message the Darwin sends to the Kinect will always be "ready". This lets the program know the Darwin is done doing its stuff
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
        
		std::string action, inputDelay, yesOrNoMovement;
		int status, index, inputDelayConverted;
		bool skip = false, repeated = false;
		
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
				
				std::vector<std::string> tokens;
				tokens = stringSplit(holder);
				
				//check that the message is the correct format (three words: "[action] [stage] [isCorrectOrNot])
				if(tokens.size() != 3)
				{
					std::cout << "Unexpected message received. Ending program." << std::endl;
					return 0;
				}
				
				action = tokens[0];
				//stage = tokens[1];
				inputDelay = tokens[1];
				yesOrNoMovement = tokens[2];

				std::cout << "Action received was " << action << "." << std::endl;

				//add a slight delay so that the robot doesn't respond too quickly
				robotwait(0.75);
				
				//the following actions preempt everything else (don't care about delays or anything like that)
				if(action == "bodyDetected")
				{
					LinuxActionScript::PlayMP3("welcome.mp3");
					robotwait(16.0);
					messageToKinect = "Robot played welcome.mp3 to acknowledge that a body was detected.";
					skipContinueMessage = true;
				}
				else if(action == "getStarted")
				{
					LinuxActionScript::PlayMP3("ready_to_begin.mp3");
					robotwait(9.0);
					previousState = "getStarted";
					messageToKinect = "Robot played ready_to_begin.mp3 to acknowledge start.";
					skipContinueMessage = true;
				}
				else if(action == "changingTwo")
				{
					LinuxActionScript::PlayMP3("phase2_ready.mp3");
					robotwait(3.0);
					messageToKinect = "Robot played phase2_ready.mp3 to acknowledge shift to second iteration.";
				}
				else if(action == "changingThree")
				{
					LinuxActionScript::PlayMP3("phase3_ready.mp3");
					robotwait(3.0);
					messageToKinect = "Robot played phase3_ready.mp3 to acknowledge shift to third iteration.";
				}
				else if(action == "nothing")
				{
					robotwait(2.0); //then well....do nothing
					messageToKinect = "Ready.";
				}
				else if(action == "repeated")
				{
					repeated = true;
					LinuxActionScript::PlayMP3("repeatedStepAlert.mp3");
					robotwait(3.0);
					messageToKinect = "Ready.";
				}
				else if(action == "forceDefault")
				{
					defaultposition();
					previousState = "default";
					robotwait(1.0);
				}
				else if(action == "end") //interaction is over
				{
					LinuxActionScript::PlayMP3("interaction_ end.mp3");
					robotwait(9.0);
					exit(0);
				}
				
				if(yesOrNoMovement == "yes")
					skip = false;
				else
					skip = true;

				std::stringstream convert(inputDelay);
				if(!(convert >> inputDelayConverted))
				{
					std::cout << "Could not properly convert input delay to an int" << std::endl;
					inputDelayConverted = 0;
				}

				//std::cout << "Made it after determining delays" << std::endl;
					
				//do the requested action
				if(action.substr(0,10) == "hokeyPokey")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " audio but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the hokeypokey action.";
						//need to make sure the robot is in default before it does the hokey pokey (ex. problems if the robot had one leg up prior)
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
						
						//do the hokey pokey
						doHokeyPokey();
						robotwait(1.0);
						previousState == "hokeyPokey";
					}
				}
				else if(action == "default")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
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
						messageToKinect = "Robot played " + chosen + " and performed the default action.";
						defaultposition();
						robotwait(1.0);
						previousState = "default";
					}
				}
				else if(action == "leftArmIn")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the leftArmIn action.";

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
						leftarmin();
						robotwait(1.0);

						previousState = "leftArmIn";
					}
				}
				else if(action == "leftArmShake")
				{
					robotwait(inputDelayConverted);			
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the leftArmShake action.";
					
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
					}
					
					//previousState = "leftArmShake";
				}
				else if(action == "rightArmIn")
				{
					robotwait(inputDelayConverted);
					//LinuxActionScript::PlayMP3("understood.mp3");
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the rightArmIn action.";

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
						rightarmin();
						robotwait(1.0);

						previousState = "rightArmIn";
					}
				}
				else if(action == "rightArmShake")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the rightArmShake action.";
						
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
					}
				}
				else if(action == "leftLegIn")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";
					
					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the leftLegIn action.";
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
						leftlegin();
						robotwait(1.0);

						previousState = "leftLegIn";
					}
				}
				else if(action == "leftLegShake")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the leftLegShake action.";
						
						if(!(previousState == "leftLegIn" || previousState == "leftLegShake"))
						{
							if(previousState == "rightLegIn" || previousState == "rightLegShake") 
							{
								rightlegbacktodefault();
								robotwait(1.0);
								leftlegin();
								robotwait(1.0);
							}
							else
							{
								defaultposition();
								robotwait(1.0);
								leftlegin();
								robotwait(1.0);
							}
						}

						leftankleup();
						robotwait(1.0);
						leftankledown();
						robotwait(1.0);
						leftankleup();
						robotwait(1.0);
						leftankledown();
						robotwait(1.0);
						leftankleup();
						robotwait(1.0);
						leftankledown();
						robotwait(1.0);

						previousState = "leftLegShake";
					}
				}
				else if(action == "rightLegIn")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perform an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the rightLegIn action.";

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
						rightlegin();
						robotwait(1.0);

						previousState = "rightLegIn";
					}
				}
				else if(action == "rightLegShake")
				{
					robotwait(inputDelayConverted);
					chosen = chooseUnderstoodMsg();
					LinuxActionScript::PlayMP3(chosen.c_str());
					robotwait(0.5);

					messageToKinect = "Robot played " + chosen + " but did not perfom an action.";

					if(!skip)
					{
						messageToKinect = "Robot played " + chosen + " and performed the rightLegShake action";
						
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
								rightlegin();
								robotwait(1.0);
							}
						}

						rightankleup();
						robotwait(1.0);
						rightankledown();
						robotwait(1.0);
						rightankleup();
						robotwait(1.0);
						rightankledown();
						robotwait(1.0);
						rightankleup();
						robotwait(1.0);
						rightankledown();

						previousState = "rightLegShake";
					}
				}
				
				for(int i=0; i<1000; i++)
				{
					receivedStr[i] = '\0';
				}
				
				robotwait(1.0);

				if(!repeated && skipContinueMessage == false)
				{

					if(action.substr(0,10) == "hokeyPokey")
					{
						if(action.substr(action.length() - 4) == "_LAD")
						{
							LinuxActionScript::PlayMP3("left_hand_done.mp3");
							robotwait(5.0);
							messageToKinect = messageToKinect + " Played left_hand_done.mp3 to inform participant that they finished the left arm.";
						}
						else if(action.substr(action.length() - 4) == "_RAD")
						{
							LinuxActionScript::PlayMP3("right_hand_done.mp3");
							robotwait(5.0);
							messageToKinect = messageToKinect + " Played right_hand_done.mp3 to inform participant that they finished the right arm.";
						}
						else if(action.substr(action.length() - 4) == "_LLD")
						{
							LinuxActionScript::PlayMP3("left_foot_done.mp3");
							robotwait(5.0);
							messageToKinect = messageToKinect + " Played left_foot_done.mp3 to inform participant that they finished the left leg.";
						}
						else if(action.substr(action.length() - 4) == "_RLD")
						{
							LinuxActionScript::PlayMP3("right_foot_done.mp3");
							robotwait(5.0);
							messageToKinect = messageToKinect + " Played right_foot_done.mp3 to inform participant that they finished the right leg.";
						}
					}
					else
					{
						chosen = chooseContinueMsg();
						LinuxActionScript::PlayMP3(chosen.c_str());
						robotwait(2.0);
					}

					messageToKinect = messageToKinect + " Played " + chosen + " to instruct participant to continue.";
				}
				
				robotwait(0.75); //because it's a bit fast during the runthrough so slow it down just a bit by adding another delay

				repeated = false;
				skip = false;
				skipContinueMessage = false;
				
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
	int i = std::rand() % 3; //pick random number between 0 and 2
	return understoodResponses[i];
}

std::string chooseContinueMsg()
{
	int i = std::rand() % 3; //pick random number between 0 and 2
	return continueResponses[i];
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
        cm730.WriteWord(id, MX28::P_GOAL_POSITION_L, sittingdefaultArray[id-1], 0);
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
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);
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
    
    //motor 4
    motorid = 4;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);
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

    //motor 6
    motorid = 6;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarminArray[motorid-1], 0);
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
}

void leftarmtiltout()
{
    //only care about servos 2, 4, and 6

    //motor 2
    motorid = 2;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);
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

    //motor 4
    motorid = 4;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);
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

    //motor 6
    motorid = 6;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltoutArray[motorid-1], 0);
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
}

void leftarmtiltin()
{
    //only care about servos 2, 4, and 6
    
    //motor 2
    motorid = 2;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0);
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

    //motor 4
    motorid = 4;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0);
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

    //motor 6
    motorid = 6;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftarmtiltinArray[motorid-1], 0);
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
}

void leftarmbacktodefault()
{
	//only care about servos 2, 4, and 6
    
    //motor 2
    motorid = 2;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	
	//motor 4
    motorid = 4;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}

	//motor 6
    motorid = 6;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
}

void rightarmin()
{
    //only care about servos 1, 3, and 5

    //motor 1
   	motorid = 1;
   	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0);
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

   	//motor 3
	motorid = 3;
   	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0);  
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

   	//motor 5
   	motorid = 5;
   	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarminArray[motorid-1], 0); 
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
}

void rightarmtiltout()
{
    //only care about servos 1, 3, and 5

    //motor 1
	motorid = 1;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0); 
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

    //motor 3
	motorid = 3;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0); 
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

    //motor 5
	motorid = 5;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltoutArray[motorid-1], 0); 
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
}

void rightarmtiltin()
{
    //only care about servos 1, 3, and 5

    //motor 1
	motorid = 1;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0); 
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

    //motor 3
	motorid = 3;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0);
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

    //motor 5
	motorid = 5;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightarmtiltinArray[motorid-1], 0);
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
}

void rightarmbacktodefault()
{
	//only care about servos 1, 3, and 5
    
    //motor 1
    motorid = 1;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	
	//motor 3
    motorid = 3;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}

	//motor 5
    motorid = 5;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
}

/*----------------------------------------------
LEG PROGRAMMING
----------------------------------------------*/
void leftlegin()
{

	//lock the all of the motors for the right leg first for support
	for(int i=7; i<=17; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, sittingdefaultArray[i-1], 0);
	}

	//only care about motors 12, 14, and 16

	//motor 12
	motorid = 12;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0); 
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
	
    //motor 14
	motorid = 14;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0);
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

    //motor 16
	motorid = 16;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftlegliftedArray[motorid-1], 0);
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
}

void leftankleup()
{
	motorid = 16;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftankleupArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftankleupArray[motorid-1])
    {
		for(int i=tmp;i>leftankleupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftankleupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void leftankledown()
{
	motorid = 16;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, leftankledownArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= leftankledownArray[motorid-1])
    {
		for(int i=tmp;i>leftankledownArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<leftankledownArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void leftlegbacktodefault()
{
	//only care about motors 12, 14, and 16

	//motor 16
	motorid = 16;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	
	robotwait(0.5); //delay to make sure the ankle goes back to the proper position before the rest of the leg moves

	//motor 12
	motorid = 12;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

    //motor 14
    motorid = 14;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void rightlegin()
{
	//lock the all of the motors for the left leg first for support
	for(int i=8; i<=18; i+=2)
	{
		cm730.WriteWord(i, MX28::P_GOAL_POSITION_L, sittingdefaultArray[i-1], 0);
	}

	//only care about motors 11, 13, and 15

	//motor 11
	motorid = 11;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0); 
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

	//motor 13
	motorid = 13;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0);
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

    //motor 15
	motorid = 15;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightlegliftedArray[motorid-1], 0); 
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
}

void rightankleup()
{
	motorid = 15;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightankleupArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightankleupArray[motorid-1])
    {
		for(int i=tmp;i>rightankleupArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightankleupArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void rightankledown()
{
	motorid = 15;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, rightankledownArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= rightankledownArray[motorid-1])
    {
		for(int i=tmp;i>rightankledownArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<rightankledownArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void rightlegbacktodefault()
{
	//only care about motors 11, 13, and 15

	//motor 15
	motorid = 15;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
	
	robotwait(0.5); //delay to make sure the ankle goes back to the proper position before the rest of the leg moves

	//motor 11
	motorid = 11;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

    //motor 13
	motorid = 13;
    //cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

//HOKEY POKEY HANDS UP CODE
void handsup()
{
	//only care about motors 1-6
	//go by arms at a time though

	//motor 1
	motorid = 1;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0); 
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

	//motor 3
	motorid = 3;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0); 
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

	//motor 5
	motorid = 5;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0); 
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

	//motor 2
	motorid = 2;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0); 
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

	//motor 4
	motorid = 4;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);
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

	//motor 6
	motorid = 6;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsupArray[motorid-1], 0);
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
}

void handsleft()
{
	//only care about motors 3 and 4 (left and right shoulders)

	//motor 3
	motorid = 3;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsleftArray[motorid-1], 0);
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

	//motor 4
	motorid = 4;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsleftArray[motorid-1], 0);
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
}

void handsright()
{
	//only care about motors 3 and 4
	
	//motor 3
	motorid = 3;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsrightArray[motorid-1], 0);
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
	
	//motor 4
	motorid = 4;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, handsrightArray[motorid-1], 0);
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
}

void handsupbacktodefault()
{
	//only care about motors 1-6

	//motor 1
	motorid = 1;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

	//motor 3
	motorid = 3;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

	//motor 5
	motorid = 5;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

	//motor 2
	motorid = 2;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}  

	//motor 4
	motorid = 4;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0); 
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 

	//motor 6
	motorid = 6;
	//cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, sittingdefaultArray[motorid-1], 0);
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	} 
}

void bendforward()
{
	//only care about motors 11 and 12

	motorid = 11;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= bentforwardArray[motorid-1])
    {
		for(int i=tmp;i>bentforwardArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<bentforwardArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}

    motorid = 12;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= bentforwardArray[motorid-1])
    {
		for(int i=tmp;i>bentforwardArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<bentforwardArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
}

void bendforwardbacktodefault()
{
	//only care about motors 11 and 12

	motorid = 11;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}

    motorid = 12;
    cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
    if(tmp >= sittingdefaultArray[motorid-1])
    {
		for(int i=tmp;i>sittingdefaultArray[motorid-1];i=i-theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
			//robotwait(0.01);
			usleep(1);
		}
	}
	else
	{
		for(int i=tmp;i<sittingdefaultArray[motorid-1];i=i+theta)
		{
			cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0); 
			//robotwait(0.01);
			usleep(1);
		}
	}
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
//looking up (Also default): 2500

void lookup()
{
	motorid = 20;
	cm730.ReadWord(motorid, MX28::P_PRESENT_POSITION_L, &tmp, 0);
	for(int i=tmp; i<sittingdefaultArray[motorid-1]; i=i+theta)
	{
		cm730.WriteWord(motorid, MX28::P_GOAL_POSITION_L, i, 0);
		usleep(1);
	}
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
}
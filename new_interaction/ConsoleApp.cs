//derived from http://stackoverflow.com/questions/5923767/simple-state-machine-example-in-c

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Text;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.IO;
using System.Media;
using Microsoft.Kinect;

namespace SkeletonDataServer
{
	class SavingKinectData
	{
		/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		VARIABLES 
		%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

		//change this variable for each subject
		public static string subjectNum = "1";

        public static string ipaddress = "10.11.130.183";

        public bool inTutorial = true; //change this in case the program crashes and you don't want to start over from the very beginning
        public bool tutorialStarted = false;

		//to be used in the output file transcribing the events during the interaction
		public string timeStamp = "";

		/**********
		* Names: 	getTimeStamp(DateTime value)
		* Purpose:	Formats the date into something more human-readable
		 **********/
		public static String getTimeStamp(DateTime value)
        {
            return value.ToString("MM-dd-yyyy__HH-mm-ss");
        }

        //output file for a specific subject
        static string filename = @"C:\Users\ti\Desktop\Subject_" + subjectNum +  ".txt";

        StreamWriter file = new StreamWriter(filename, true);

        /*Symbolic constants for robot within allMoves array (array that will be declared later
        that keeps track of when a robot should make a mistake during the dance)*/
        const int DONTMAKEERROR = 0;
        const int MAKEERROR = 1;

        /*Symbolic constants for which error state [many mistakes] -> [fewer mistakes] -> [very very few mistakes]*/
        const int STAGE0PREINTERACTION = 0;
        const int STAGE1MANYMISTAKES = 1;
        const int STAGE2FEWMISTAKES = 2;
        const int STAGE3NONEISHMISTAKES = 3;

        //Kinect for Windows V2 Sensor
        public KinectSensor sensor;

        //frame readers for processing synchronized streams
        private BodyFrameReader bodyReader;

        //array updated with new frame data when it becomes availalbe
        private Body[] bodyData = null;

        //incremented each time body frame has arrived
        private int frameNumber;

        //create a lock so that multiple events don't try to do the same thing at once (send data to Darwin server)
        private Object thisLock = new Object();
        
        //this variable is modified in the lock (set to true and then false after two-way communication), when set to true event handlers do not process body frame data
        private bool processing = false;

        //the message that is sent to the Darwin server (a string with two words separated by a space)
        private string dataToSend = "";

        int counter = 0, invalidCounter = 0;

        bool welcomeFlag = false;

        //response from the Darwin server 
        private string response = "";

        //set to true when the Kinect client successfully connects to the Darwin server
        private bool connected = false;

        //set to true when this program has data to send to the Kinect
        private bool readyToWrite = false;

        //useful to prevent multiple event handlers from trying to write to the readyToWrite variable
        private bool dataBeingPrepped = false;

        //25 tracked joint names in the order of the JointType enum, comma separated
        private string jointNames = null;

        //array holding the joint information (joint names and joint data)
        //25 for the number of joints
        //9 for the jointName and then the 8 data points
        private string[,] jointNamesAndData = new string[25,9];

        static int delay = 1;

        //variables needed for FPS calculation
        private System.Diagnostics.Stopwatch frameTime = null;
        private DateTime nextStatusUpdate;
        private DateTime nextFrameCaptureUpdate = DateTime.Now + TimeSpan.FromSeconds(delay);
        private uint framesSinceUpdate;

        //set to DateTime.Now when first frame arrives, used to calculate how much time has elapsed since the
        //beginning of the KStudio recording and find the timestamp with respect to the KStudio filename
        private DateTime startTime;

        private long bodyStartTime = 0;

        //this variable checks to see when the interaction begins (with person holding both arms out to the side)
        bool started = false;

        //this variable checks for the last hokeypokey done in the third iteration to end the interaction
        bool ended = false;

        bool skipLogic = false;

        //variables needed to give participants one free accidental move after doing a limb shake during the hokeypokey sequence
        //to avoid the state machine jumping to a previous state and then to the non-sequential hokey pokey if they attempt the hokey pokey after that
        bool leftLegFreebie = false, rightLegFreebie = false, leftArmFreebie = false, rightArmFreebie = false;

        //keep track of previous position, especially useful for determining if a hand or foot is shaking
        string previousPosition = "default";

        //variable to keep track of the active limb that the participant is demonstrating 
        //for example: if the participant says they are teaching the movements for the left arm but then toss in the right arm, then the robot can point this out
        string activeLimb = "none";

        //record the last three positions where if they're all equal, then the teacher is done with the demonstration
        string[] pastPositions = new string[3];
        int pastPositionsCounter = 0;

        string incorrectAction;

        string yesOrNoMovement;

        int movementDelay;

        string audioclip;

        //variables needed to determine if there is any hand or foot shaking
        double leftFootLastX = 0.0, rightFootLastX = 0.0;
        double leftHandLastX = 0.0, rightHandLastX = 0.0;

        //random number needed for the three test conditions
        Random random = new Random();
        int stateCount = STAGE0PREINTERACTION, numMistakes, randomNum, tempRandom, correctClip;

        //string[] incorrectMoveArray = {"leftArm", "rightArm", "leftLeg", "rightLeg", "default"};
        string[] incorrectMoveArray = {"leftArm", "rightArm", "leftLeg", "rightLeg"};

        string[] limbOrder = new string[4];
        string[] leftArmMovements = new string[100]; //don't need this many slots but I'm just covering all my bases
        string[] rightArmMovements = new string[100];
        string[] leftLegMovements = new string[100];
        string[] rightLegMovements = new string[100];

        int limbCounter = 0;
        int leftArmCounter = 0, rightArmCounter = 0, leftLegCounter = 0, rightLegCounter = 0;

        bool darwinWasCorrect = false, inIncorrectnessConfirmation = false, inIncorrectLimbQuery = false, inNextLimbQuery = false, hokeypokeyDone = false, waitingToConfirm = false, limbDone = false;

        string messageToDarwin = "", moveSequence = "";

        /**********
        * Name: 		determineIncorrectMove
        * Purpose:		To randomly pick the wrong action to perform when the robot makes a mistake
        				In order to make sure it's perceived as a mistake, don't do a move on the same
        				limb (ex. if a robot is actually supposed to put right arm in, have the mistake
        				be an action done by any other limb)
        * Parameters: 	A string that is either leftArm/rightArm/leftLeg/rightLeg/default
         **********/
        public string determineIncorrectMove(string temp)
        {
            bool tempFlag = true;
            int choice = -1;
            string wrongAction = "";
            while(tempFlag)
            {
                choice = random.Next(0,4);
                if(incorrectMoveArray[choice] != temp)
                {
                    tempFlag = false;
                }
            }

            //if(!incorrectMoveArray[choice].Equals("default"))
            wrongAction = incorrectMoveArray[choice] + "In";

           return wrongAction;

           //string wrongAction = "spaz";
           //return wrongAction;
        }

        public void determineAudioPrompt(string temp)
        {
        	switch(temp)
        	{
        		case("DEFAULT_STATE_22"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\lefthandin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_HAND_IN_2"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\lefthandout_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_HAND_OUT_3"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\lefthandin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_HAND_IN_4"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\lefthandshake_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_HAND_SHAKE_5"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\hokeypokey_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;	
        		case("HOKEY_POKEY_6"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\righthandin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_HAND_IN_7"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\righthandout_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_HAND_OUT_8"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\righthandin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_HAND_IN_9"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\righthandshake_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_HAND_SHAKE_10"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\hokeypokey_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("HOKEY_POKEY_11"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\leftfootin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_LEG_IN_12"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\leftfootout_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_LEG_OUT_13"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\leftfootin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_LEG_IN_14"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\leftfootshake_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("LEFT_LEG_SHAKE_15"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\hokeypokey_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("HOKEY_POKEY_16"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\rightfootin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_LEG_IN_17"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\rightfootout_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_LEG_OUT_18"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\rightfootin_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_LEG_IN_19"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\rightfootshake_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        		case("RIGHT_LEG_SHAKE_20"):
        			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\hokeypokey_tutorial.wav"))
					{
						simpleSound.PlaySync();
					}
        			break;
        	}
        }

        //variable to keep track of current stage
        //string stage = "";

        //array for the moves 
        //There are 20 nodes that we care whether or not there is an error associated with it 
        //However there's an additional six nodes (last six in DanceState) for non sequential moves that we also care about, but these nodes will never have errors assigned
        //we disregard the first two states PRE_INTERACTION and PURGATORY, so that's why all of the MAKEERROR checks in the code have an offset of -2
        int[] allMoves = new int[26] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        //array to let you know when the end of the branch is reached (lArm, rArm, lLeg, rLeg)
        //need to keep track of this to know when to go to the next iteration of the dance
        int[] branchTracker = new int[4] { 0, 0, 0, 0 };
        const int BRANCH_NOT_DONE = 0, BRANCH_DONE = 1;

        //variable that keeps track of when to change state (and therefore when to retrieve a new random number for the stage)
        bool changingState = false;

        //needed so that the robot can state when a branch has finished
        bool informLeftLegDone = false, informRightLegDone = false, informLeftArmDone = false, informRightArmDone = false;

		//all of the [x-limb]_OUTs are the same as default
		public enum DanceState 
		{ 
			PRE_INTERACTION_0, 
			PURGATORY_1, 
			LEFT_HAND_IN_2, 
			LEFT_HAND_OUT_3, 
			LEFT_HAND_IN_4, 
			LEFT_HAND_SHAKE_5, 
			HOKEY_POKEY_6, 
			RIGHT_HAND_IN_7, 
			RIGHT_HAND_OUT_8, 
			RIGHT_HAND_IN_9, 
			RIGHT_HAND_SHAKE_10, 
			HOKEY_POKEY_11, 
			LEFT_LEG_IN_12, 
			LEFT_LEG_OUT_13, 
			LEFT_LEG_IN_14, 
			LEFT_LEG_SHAKE_15, 
			HOKEY_POKEY_16, 
			RIGHT_LEG_IN_17, 
			RIGHT_LEG_OUT_18, 
			RIGHT_LEG_IN_19, 
			RIGHT_LEG_SHAKE_20, 
			HOKEY_POKEY_21, 
			DEFAULT_STATE_22,
			HOKEY_POKEY_NON_SEQUENTIAL_23,
			LEFT_HAND_SHAKE_NON_SEQUENTIAL_24,
			RIGHT_HAND_SHAKE_NON_SEQUENTIAL_25,
			LEFT_LEG_SHAKE_NON_SEQUENTIAL_26,
			RIGHT_LEG_SHAKE_NON_SEQUENTIAL_27
		}

		public enum DanceMove 
		{ 
			BHO, //both hands out to start the interaction
			LHI, 
			LHS, 
			RHI, 
			RHS, 
			LLI, 
			LLS, 
			RLI, 
			RLS,
			HP,
			DEFAULT, 
			INVALID,
		}

		/* Class Names: DanceState and StateTransition
		 * Purpose:		DanceState - represents the all of the different states that can occur during the interaction
		 				StateTransition - represents the transitions between states when various actions happen
		*/
		class Dance
		{
			class StateTransition
			{
				readonly DanceState CurrentState;
				readonly DanceMove DanceMove;

				public StateTransition(DanceState currentState, DanceMove danceMove)
				{
					CurrentState = currentState;
					DanceMove = danceMove;
				}

				public override int GetHashCode() //something about avoiding dictionary collisions and optimizing: http://stackoverflow.com/questions/5923767/simple-state-machine-example-in-c
				{
					return 17 + 31 * CurrentState.GetHashCode() + 31 * DanceMove.GetHashCode();
				}

				public override bool Equals(object obj)
				{
					StateTransition other = obj as StateTransition;
					return other != null && this.CurrentState == other.CurrentState && this.DanceMove == other.DanceMove;
				}
			}

			Dictionary<StateTransition, DanceState> transitions;
			public DanceState CurrentState {get; private set; }

			public Dance()
			{
				//I need purgatory to serve an an auto-forward state (where if it's reached, then it immediately jumps to a different state)
				CurrentState = DanceState.PRE_INTERACTION_0;
				transitions = new Dictionary<StateTransition, DanceState>
				{
					{ new StateTransition(DanceState.PRE_INTERACTION_0, DanceMove.BHO), DanceState.DEFAULT_STATE_22 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.LHI), DanceState.LEFT_HAND_IN_2 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.RHI), DanceState.RIGHT_HAND_IN_7 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.LLI), DanceState.LEFT_LEG_IN_12 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.RLI), DanceState.RIGHT_LEG_IN_17 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.DEFAULT), DanceState.DEFAULT_STATE_22 },
					{ new StateTransition(DanceState.PURGATORY_1, DanceMove.HP), DanceState.HOKEY_POKEY_NON_SEQUENTIAL_23 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_2, DanceMove.LHI), DanceState.LEFT_HAND_IN_2 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_2, DanceMove.DEFAULT), DanceState.LEFT_HAND_OUT_3 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_2, DanceMove.LHS), DanceState.LEFT_HAND_SHAKE_NON_SEQUENTIAL_24 },	//if the participant shakes their hand too soon
					{ new StateTransition(DanceState.LEFT_HAND_OUT_3, DanceMove.DEFAULT), DanceState.LEFT_HAND_OUT_3 },
					{ new StateTransition(DanceState.LEFT_HAND_OUT_3, DanceMove.LHI), DanceState.LEFT_HAND_IN_4 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_4, DanceMove.LHI), DanceState.LEFT_HAND_IN_4 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_4, DanceMove.DEFAULT), DanceState.LEFT_HAND_OUT_3 },
					{ new StateTransition(DanceState.LEFT_HAND_IN_4, DanceMove.LHS), DanceState.LEFT_HAND_SHAKE_5 },
					{ new StateTransition(DanceState.LEFT_HAND_SHAKE_5, DanceMove.LHS), DanceState.LEFT_HAND_SHAKE_5 },
					{ new StateTransition(DanceState.LEFT_HAND_SHAKE_5, DanceMove.LHI), DanceState.LEFT_HAND_IN_4 },
					{ new StateTransition(DanceState.LEFT_HAND_SHAKE_5, DanceMove.DEFAULT), DanceState.LEFT_HAND_SHAKE_5 },
					{ new StateTransition(DanceState.LEFT_HAND_SHAKE_5, DanceMove.HP), DanceState.HOKEY_POKEY_6 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_7, DanceMove.RHI), DanceState.RIGHT_HAND_IN_7 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_7, DanceMove.DEFAULT), DanceState.RIGHT_HAND_OUT_8 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_7, DanceMove.RHS), DanceState.RIGHT_HAND_SHAKE_NON_SEQUENTIAL_25 },	//if the participant shakes their hand too soon
					{ new StateTransition(DanceState.RIGHT_HAND_OUT_8, DanceMove.DEFAULT), DanceState.RIGHT_HAND_OUT_8 },
					{ new StateTransition(DanceState.RIGHT_HAND_OUT_8, DanceMove.RHI), DanceState.RIGHT_HAND_IN_9 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_9, DanceMove.RHI), DanceState.RIGHT_HAND_IN_9 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_9, DanceMove.DEFAULT), DanceState.RIGHT_HAND_OUT_8 },
					{ new StateTransition(DanceState.RIGHT_HAND_IN_9, DanceMove.RHS), DanceState.RIGHT_HAND_SHAKE_10 },
					{ new StateTransition(DanceState.RIGHT_HAND_SHAKE_10, DanceMove.RHS), DanceState.RIGHT_HAND_SHAKE_10 },
					{ new StateTransition(DanceState.RIGHT_HAND_SHAKE_10, DanceMove.RHI), DanceState.RIGHT_HAND_IN_9 },
					{ new StateTransition(DanceState.RIGHT_HAND_SHAKE_10, DanceMove.DEFAULT), DanceState.RIGHT_HAND_SHAKE_10 },
					{ new StateTransition(DanceState.RIGHT_HAND_SHAKE_10, DanceMove.HP), DanceState.HOKEY_POKEY_11 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_12, DanceMove.LLI), DanceState.LEFT_LEG_IN_12 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_12, DanceMove.DEFAULT), DanceState.LEFT_LEG_OUT_13 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_12, DanceMove.LLS), DanceState.LEFT_LEG_SHAKE_NON_SEQUENTIAL_26 }, 	//if the participant shakes their leg too soon
					{ new StateTransition(DanceState.LEFT_LEG_OUT_13, DanceMove.DEFAULT), DanceState.LEFT_LEG_OUT_13 },
					{ new StateTransition(DanceState.LEFT_LEG_OUT_13, DanceMove.LLI), DanceState.LEFT_LEG_IN_14 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_14, DanceMove.LLI), DanceState.LEFT_LEG_IN_14 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_14, DanceMove.DEFAULT), DanceState.LEFT_LEG_OUT_13 },
					{ new StateTransition(DanceState.LEFT_LEG_IN_14, DanceMove.LLS), DanceState.LEFT_LEG_SHAKE_15 },
					{ new StateTransition(DanceState.LEFT_LEG_SHAKE_15, DanceMove.LLS), DanceState.LEFT_LEG_SHAKE_15 },
					{ new StateTransition(DanceState.LEFT_LEG_SHAKE_15, DanceMove.LLI), DanceState.LEFT_LEG_IN_14 },
					{ new StateTransition(DanceState.LEFT_LEG_SHAKE_15, DanceMove.DEFAULT), DanceState.LEFT_LEG_SHAKE_15 },
					{ new StateTransition(DanceState.LEFT_LEG_SHAKE_15, DanceMove.HP), DanceState.HOKEY_POKEY_16 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_17, DanceMove.RLI), DanceState.RIGHT_LEG_IN_17 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_17, DanceMove.DEFAULT), DanceState.RIGHT_LEG_OUT_18 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_17, DanceMove.RLS), DanceState.RIGHT_LEG_SHAKE_NON_SEQUENTIAL_27 }, 	//if the participant shakes their leg too soon
					{ new StateTransition(DanceState.RIGHT_LEG_OUT_18, DanceMove.DEFAULT), DanceState.RIGHT_LEG_OUT_18 },
					{ new StateTransition(DanceState.RIGHT_LEG_OUT_18, DanceMove.RLI), DanceState.RIGHT_LEG_IN_19 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_19, DanceMove.RLI), DanceState.RIGHT_LEG_IN_19 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_19, DanceMove.DEFAULT), DanceState.RIGHT_LEG_OUT_18 },
					{ new StateTransition(DanceState.RIGHT_LEG_IN_19, DanceMove.RLS), DanceState.RIGHT_LEG_SHAKE_20 },
					{ new StateTransition(DanceState.RIGHT_LEG_SHAKE_20, DanceMove.RLS), DanceState.RIGHT_LEG_SHAKE_20 },
					{ new StateTransition(DanceState.RIGHT_LEG_SHAKE_20, DanceMove.RLI), DanceState.RIGHT_LEG_IN_19 },
					{ new StateTransition(DanceState.RIGHT_LEG_SHAKE_20, DanceMove.DEFAULT), DanceState.RIGHT_LEG_SHAKE_20 },
					{ new StateTransition(DanceState.RIGHT_LEG_SHAKE_20, DanceMove.HP), DanceState.HOKEY_POKEY_21 },
				};
			}

			public DanceState GetNext(DanceMove danceMove)
			{
				StateTransition transition = new StateTransition(CurrentState, danceMove);
				DanceState nextState;
				if(!transitions.TryGetValue(transition, out nextState))
				{
					//looks like purgatory works best as serving as the state right after the person begins the interaction
					//by holding both hands out, and then waiting for the next dance

					//now depending on the danceMove send it to a different tree
					if(danceMove == DanceMove.LHI || danceMove == DanceMove.LHS)
						nextState = DanceState.LEFT_HAND_IN_2; 		//send it to the beginning of the left hand tree
                    else if (danceMove == DanceMove.RHI || danceMove == DanceMove.RHS)
                        nextState = DanceState.RIGHT_HAND_IN_7; 	//send it to the beginning of the right hand tree
                    else if (danceMove == DanceMove.LLI || danceMove == DanceMove.LLS)
                        nextState = DanceState.LEFT_LEG_IN_12;  	//send it to the beginning of the left leg tree
                    else if (danceMove == DanceMove.RLI || danceMove == DanceMove.RLS)
                        nextState = DanceState.RIGHT_LEG_IN_17;		//send it to the beginning of the right leg tree
                    else if (danceMove == DanceMove.DEFAULT)
                        nextState = DanceState.DEFAULT_STATE_22;
                    else if (danceMove == DanceMove.HP)
                        nextState = DanceState.HOKEY_POKEY_NON_SEQUENTIAL_23;
					else 											//for an invalid move 					
                        nextState = DanceState.PURGATORY_1;

                    //note that there are no transitions for the hokey pokey at the end of each limb branch in the state machine
            		//so any move done at those states will automatically shift the current state away from those end points
				}
				return nextState;
			}

			public DanceState MoveNext(DanceMove danceMove)
			{
				CurrentState = GetNext(danceMove);
				return CurrentState;
			}
		}

        //set up the Dance state machine
        Dance theHokeyPokeyDance = new Dance();

		/*	Function Name:	SavingKinectData
		* 	Parameters:		none
		* 	Purpose: 		Class constructor - initializes kinect and buffers, initializes connection to Darwin, and
		*					subscribes to Kinect body data event handler
		*/
		public SavingKinectData()
		{
			//intialize data for fps calculation
            this.frameTime = new System.Diagnostics.Stopwatch();
            this.nextStatusUpdate = DateTime.MinValue;
            this.framesSinceUpdate = 0;

            //map enum names of joints to openni joint names in enum order
            this.jointNames = mapJointNamesToOpenNI();

            //first frame read has frameNumber = 0 
            this.frameNumber = 0;

            //initialize kinect and data buffers
            this.sensor = KinectSensor.GetDefault();
            if (this.sensor != null)
            {
                //open sensor
                this.sensor.Open();

                //determine dimensions needed for each of the arrays of data using FrameDescription/ BodyCount
                this.bodyData = new Body[this.sensor.BodyFrameSource.BodyCount];

                this.bodyReader = this.sensor.BodyFrameSource.OpenReader();

                //subscribe to frame arrival event, event raised when a new body frame arrives
                this.bodyReader.FrameArrived += bodyReader_FrameArrived;
                
                //Establishing connection
            	Socket clientSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            	//EndPoint sendEndPoint;

      			try{
                    clientSocket.Connect(ipaddress, 3333);
         			Console.WriteLine("Connected to Darwin");
            		connected = true;
      			} catch (SocketException e)
      			{
         			Console.WriteLine("Unable to connect to Darwin.");
         			Console.WriteLine(e.ToString());
         			return;
      			}

      			//this variable is just used once so that the program can state when the main loop of the program starts after a successful connection
      			bool flag = true;
      			
      			try{
      			    //loop for interaction with the server
                    while(connected)
                    {
                        if(flag)
                        {
                        	if(inTutorial)
                        	{
                        		Console.WriteLine("------IN TUTORIAL PROGRAM------");
                        		Console.WriteLine("Entered while loop for interaction with the Kinect");
                        		Console.WriteLine("\tJust started. Current state = " + theHokeyPokeyDance.CurrentState);
                        		file.WriteLine("Just started. Current state = " + theHokeyPokeyDance.CurrentState + ". Time: " + getTimeStamp(DateTime.Now));
                        	}
                        	else
                        	{
                            	//this stuff is shown once
                            	Console.WriteLine("------IN MAIN PROGRAM-----");
                            	Console.WriteLine("Entered while loop for interaction with the server");
                            	Console.WriteLine("\tJust started. Current state = " + theHokeyPokeyDance.CurrentState);
                            	file.WriteLine("Just started. Current state = " + theHokeyPokeyDance.CurrentState + ". Time: " + getTimeStamp(DateTime.Now));
                            }
                            flag = false;
                        }

                        //event handlers check the "processing" variable before attempting to process code
                        //when a single event handler is clear to process data, it sets the variable to false so that other handlers do not process the code
                        //the variable is not set to false until after the two-way communication occurs
                        lock(thisLock)
                        {
                            //Console.WriteLine("Inside locked section of code");
                            if (!processing && readyToWrite) //need to send stuff through socket if nothing else is being processed 
                            {
                                processing = true; //so other event handlers after this will read this value and not do the body processing

                                /*inTutorial = false;
								tutorialStarted = false;
								stateCount = 0; // reset the state for the Darwin portion of the program*/
                                if(inTutorial)
                                {
                                	string[] words = dataToSend.Split(' ');
                                	if(!words[0].Equals("nothing"))
                                	{
                                		if(words[0].Equals("end"))
                                		{
                                			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\correct_tutorial.wav"))
											{
												simpleSound.PlaySync();
											}
                                			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\newend_tutorial.wav"))
											{
												simpleSound.PlaySync();
											}
                                			inTutorial = false;
											tutorialStarted = false;
											stateCount = 0; // reset the state for the Darwin portion of the program*/
                                		}
                                		else if(words[0].Equals("bodyDetected"))
                                		{
                                			using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\welcome_tutorial.wav"))
											{
												simpleSound.PlaySync();
											}
										}
										else if(words[0].Equals("getStarted"))
										{
											using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\interactionstart_tutorial.wav"))
											{
												simpleSound.PlaySync();
											}
                                            using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\lefthandin_tutorial.wav"))
                                            {
                                                simpleSound.PlaySync();
                                            }
										}
										else //everything else is a dance move
										{
                                			if(words[0].Equals("correct"))
                                			{
                                                correctClip = random.Next(0,3);
                                                if(correctClip == 0)
                                                {
                                				    using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\yescorrect_tutorial.wav"))
												    {
													   simpleSound.PlaySync();
												    }
                                                }
                                                else if(correctClip == 1)
                                                {
                                                    using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\thatiscorrect_tutorial.wav"))
                                                    {
                                                       simpleSound.PlaySync();
                                                    }
                                                }
                                                else
                                                {
                                                    using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\correctmove_tutorial.wav"))
                                                    {
                                                       simpleSound.PlaySync();
                                                    }
                                                }
                                			}
                                			else
                                			{
                                				using (SoundPlayer simpleSound = new SoundPlayer(@"C:\Users\ti\Downloads\tutorialaudio\incorrect_tutorial.wav"))
												{
													simpleSound.PlaySync();
												}
                                			}

                                			determineAudioPrompt(words[1]);
                                		}
                                        System.Threading.Thread.Sleep(2500);
                                   		//Console.WriteLine("Kinect played \"" + audioclip + "\". Time: " + getTimeStamp(DateTime.Now));
                                	}
                                }
                                else //in actual LfD portion
                                {
                                    if(!dataToSend.Equals("nothing here alright"))
                                    {
                                        if(dataToSend.Equals("sequence"))
                                        {
                                            //the message sent to the Darwin will be the move sequence
                                            //messageToDarwin = moveSequence;
                                            messageToDarwin = createMovementSequence();
                                            waitingToConfirm = true;
                                        }
                                        else if(dataToSend.Equals("repeated"))
                                        {
                                            //tell the Darwin that the participant did a repeated move
                                            messageToDarwin = "repeated";
                                        }
                                        else if(dataToSend.Equals("humanError"))
                                        {
                                            messageToDarwin = "humanError";
                                        }
                                        else if(dataToSend.Equals("wrongconfirmation"))
                                        {
                                            //tell the Darwin that it either did the correct or incorrect thing after it attempts the dance
                                            messageToDarwin = "incorrect";
                                            inIncorrectLimbQuery = true;
                                        }
                                        else if(dataToSend.Equals("rightconfirmation"))
                                        {
                                            messageToDarwin = "correct"; //darwin will ask what limb the person will teach next
                                            inNextLimbQuery = true;
                                        }
                                        else if(dataToSend.Equals("newLimb"))
                                        {
                                            messageToDarwin = "newLimb"; //darwin will tell the person to begin
                                        }
                                        else if(dataToSend.Equals("repeatLimb"))
                                        {
                                            messageToDarwin = "repeatLimb";
                                        }
                                        else if(dataToSend.Equals("getStarted"))
                                        {
                                            messageToDarwin = "getStarted";
                                        }
                                        else //that it was just an action performed
                                        {
                                            messageToDarwin = "actionPerformed";
                                        }

                                    	//Console.WriteLine("Processing has started");
                                    	byte[] outStream = System.Text.Encoding.ASCII.GetBytes(messageToDarwin);
                                    	clientSocket.Send(Encoding.ASCII.GetBytes(messageToDarwin));
                                    	Console.WriteLine("Sent data: " + messageToDarwin);
                                    	file.WriteLine("Sent the following to robot: " + messageToDarwin + ". Time: " + getTimeStamp(DateTime.Now));

                                    	byte[] bytes = new byte[256];
                                    	int i = clientSocket.Receive(bytes); //Cory note: may need to change this code to actually make sure we recieved a message
                                    	response = Encoding.UTF8.GetString(bytes);
                                    	//Console.WriteLine("Received data: " + response);
                                    	Console.WriteLine("Received message from robot: " + response);
                                    	file.WriteLine("Robot completed action and sent response: " + response + " Time: " + getTimeStamp(DateTime.Now));
                                    	file.WriteLine("");
                                    	file.Flush();
                                	}
                                }
                        
                                processing = false; //now future event handlers can start processing again
                                readyToWrite = false;
                                dataBeingPrepped = false;
                            }
                            //Console.WriteLine("Exiting locked section of code");
                        }
      			   }
      			   file.Close();
      			   connected = false;
      			   Console.WriteLine("Disconnected from Darwin.");
      		    } catch(Exception e)
      		    {
      			   Console.WriteLine(e.Message);
      		    }
      		}
		}

		/* Function Name:   bodyReader_FrameArrived
         * Parameters:      sender-object raising the event, e-event arguments
         * Returns:         void
         * Purpose:         Event handler for arrival of frames from body stream.
         */
        void bodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            //fps calculation
            //calculateFPS();

            //when first frame arrives, record start time
            if (this.frameNumber == 0)
            {
                this.startTime = DateTime.Now;
            }

            /*****body frame processing*****/
            using (BodyFrame frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (this.bodyStartTime == 0)
                    {
                        //bodyStartTime takes into account time elapsed during initial null frames
                        this.bodyStartTime = frame.RelativeTime.Ticks - (DateTime.Now - this.startTime).Ticks;
                    }

                    long stamp = getTicksSinceEpoch(frame.RelativeTime.Ticks, this.bodyStartTime);

                    long secs = stamp / 10000000;     //10 million ticks / second
                    long nsecs = (stamp % 10000000) * 100;    //100 ns / tick

                    frame.GetAndRefreshBodyData(this.bodyData);
                    foreach (Body body in this.bodyData)
                    {
                        // if no body is tracked for a frame, no data is written to file for that frame
                        //if (body.IsTracked)
                        //if (body.IsTracked && DateTime.Now >= this.nextFrameCaptureUpdate)
                        if (body.IsTracked)
                        {
                            //bodyCurrentlyTracked = true; //trying to limit how fast data is processed
                            if (connected && !dataBeingPrepped)
                            {
                                dataBeingPrepped = true;

                                //each line of csv file contains the frame number, timestamp, tracking id, and ordered list of joint names
                                string bodyLine = this.frameNumber + "," + secs + "," + nsecs + "," + (Int32)body.TrackingId;
                                string bodyLine2 = this.jointNames;
                                string confidence = null;

                                string[] temp = bodyLine2.Split(',');
                                for (int i = 0; i < temp.Length; i++)
                                {
                                    jointNamesAndData[i, 0] = temp[i];
                                }

                                int numJoints = 25;

                                //iterate through all joints in enum order
                                for (int i = 0; i < numJoints; ++i)
                                {
                                    Joint joint = body.Joints[(JointType)i];
                                    JointOrientation jOrient = body.JointOrientations[(JointType)i];

                                    //confidence = 0.0 for not tracked, 0.5 for inferred, 1.0 for tracked
                                    confidence = (float)((int)joint.TrackingState / 2.0) + "";

                                    jointNamesAndData[i, 1] = confidence;

                                    //position is 3 dimensional position with the depth/IR camera as origin (Camera Space Points)
                                    jointNamesAndData[i, 2] = Convert.ToString(joint.Position.X);
                                    jointNamesAndData[i, 3] = Convert.ToString(joint.Position.Y);
                                    jointNamesAndData[i, 4] = Convert.ToString(joint.Position.Z);

                                    //orienation is quaternion relative to parent joint
                                    jointNamesAndData[i, 5] = Convert.ToString(jOrient.Orientation.W);
                                    jointNamesAndData[i, 6] = Convert.ToString(jOrient.Orientation.X);
                                    jointNamesAndData[i, 7] = Convert.ToString(jOrient.Orientation.Y);
                                    jointNamesAndData[i, 8] = Convert.ToString(jOrient.Orientation.Z);
                                }

                                //data parsing before writing to another file
                                string[] header = bodyLine.Split(',');

                                //sanity check
                                //header should be 4 fields in length (frame, seconds, nano seconds, tracking id)
                                if (header.Length != 4)
                                    Console.WriteLine("Error: Header was parsed incorrectly");

                                //joints should be 25 fields in length
                                if (jointNamesAndData.GetLength(0) != 25)
                                    Console.WriteLine("Error: Joint names were parsed incorrectly");

                                //second dimension should be 9 (jointName + 8 data points) fields in length
                                if (jointNamesAndData.GetLength(1) != 9)
                                    Console.WriteLine("Error: Joint data was parsed incorrectly");

                                //another sanity check
                                if (jointNamesAndData[4, 0] != "ShoulderLeft" || jointNamesAndData[7, 0] != "HandLeft" || jointNamesAndData[8, 0] != "ShoulderRight" || jointNamesAndData[11, 0] != "HandRight" || jointNamesAndData[15, 0] != "FootLeft" || jointNamesAndData[19, 0] != "FootRight")
                                    Console.WriteLine("ERROR. Calculating wrong joints");

                                //we have all of the joint data now
                                //so it's time to determine what action the participant performed
                                if(hokeypokeyDone) //if the previous motion was hokeyPokey, then we are sending the commands to the robot
                                {
                                    dataToSend = "sequence";
                                    hokeypokeyDone = false;
                                }
                                else if((changingState == true) && (stateCount > STAGE0PREINTERACTION))
                                {
                                	//by changing state, I mean starting another iteration of the hokey pokey
                                	stateCount++;

                                	//do an invalid transition to reset the state to PURGATORY_1
                                	theHokeyPokeyDance.MoveNext(DanceMove.INVALID);

                                	//reset branchTracker array
                                	for(int k=0; k<branchTracker.Length; k++)
                                	{
                                		branchTracker[k] = BRANCH_NOT_DONE;
                                	}

                                	if(inTutorial)
                                	{
                                		Console.WriteLine("***ENDING TUTORIAL***");
                                		file.WriteLine("***ENDING TUTORIAL. Time: " + getTimeStamp(DateTime.Now));
                                		dataToSend = "end 0 done";
                                	}
                                	else if(stateCount > STAGE3NONEISHMISTAKES) //if we're already in the third iteration
                                	{
                                		ended = true;
                                		started = false;

                                		Console.WriteLine("***REACHED END OF THIRD ITERATION OF DANCE. INTERACTION OVER***");
                                		file.WriteLine("***Interaction ending. Time: " + getTimeStamp(DateTime.Now));

                                		//then the interaction is done
                                		dataToSend = "end";
                                	}
                                	else
                                	{
                                		if(stateCount == STAGE2FEWMISTAKES)
                                		{
                                			//stage = "two";
                                			//numMistakes = random.Next(0,4) + 4; //pick randomly between 4-7 mistakes
                                			if(!inTutorial)
                                				numMistakes = 7;
                                			dataToSend = "changingTwo 0 go";

                                			Console.WriteLine("***STARTING SECOND ITERATION OF DANCE***");
                                			file.WriteLine("***Starting second iteration of dance. Time: " + getTimeStamp(DateTime.Now));
                                		}
                                		else if(stateCount == STAGE3NONEISHMISTAKES)
                                		{
                                			//stage = "three";
                                			//numMistakes = random.Next(0,4); 	//pick randomly between 0-3 mistakes
                                			if(!inTutorial)
                                				numMistakes = 3;
                                			dataToSend = "changingThree 0 go";

                                			Console.WriteLine("***STARTING THIRD ITERATION OF DANCE***");
                                			file.WriteLine("***Starting third iteration of dance. Time: " + getTimeStamp(DateTime.Now));
                                		}
                                	}

                                	if(!inTutorial)
                                	{
	                               		//make sure that the allMoves array is clear
	                               		for(int i=0; i<allMoves.Length; i++)
	                               		{
	                               			allMoves[i] = DONTMAKEERROR; //DONTMAKEERROR = 0;
	                               		}

	                               		//now that we have randomly chosen the number of mistakes, we need to randomly pick which
	                               		//of the 20 distinct moves per iteration will be a mistake
	                               		int filler = 0;
	                               		while (filler < numMistakes)
	                               		{
	                               			randomNum = random.Next(0,20);

	                               			if(allMoves[randomNum] == DONTMAKEERROR)
	                               			{
	                               				//but don't make an error on a retraction ([x-limb out])
	                               				if(!(randomNum == 1 || randomNum == 6 || randomNum == 11 || randomNum == 16))
	                               				{
	                               					allMoves[randomNum] = MAKEERROR; //MAKEERROR = 1
	                               					filler++;
	                               				}
	                               			}
	                               		}

	                                    file.WriteLine("***MOVES ARRAY (0 = no mistake, 1 = mistake)***");
	                                    for (int i = 0; i < 20; i++)
	                                    {
	                                        if (i == 0)
	                                            file.Write("[" + allMoves[i] + ",");
	                                        else if (i == 19)
	                                            file.WriteLine(allMoves[i] + "]");
	                                        else
	                                            file.Write(allMoves[i] + ",");
	                                    }
	                                    file.WriteLine("***END MOVES ARRAY***");
                                	}
                               		//we've finished changing states, now update the variable
                               		changingState = false;
                               		readyToWrite = true;
                                }
                                else if(bothHandsOutToSide() && !started && !ended)
                                {
                                	if(inTutorial) //I also only want stuff done once when the tutorial starts
                                	{
                                		if(!tutorialStarted)
                                		{
	                                		previousPosition = "started";
	                                		tutorialStarted = true;

	                                		Console.WriteLine("Participant stuck out both arms to signify the beginning of interaction");
	                                		Console.WriteLine("***STARTING FIRST ITERATION OF DANCE***");
	                                		theHokeyPokeyDance.MoveNext(DanceMove.BHO);
                                         
	                                		Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

	                                		file.WriteLine("***Participant initiated interaction. Time: " + getTimeStamp(DateTime.Now));
	                                		file.WriteLine("\tCurrent dance state = " + theHokeyPokeyDance.CurrentState);

	                                    	//stateCount = STAGE1MANYMISTAKES;

	                                    	dataToSend = "getStarted 0 alright";
	                                    }
	                                    else
	                                    	dataToSend = "nothing here alright"; //otherwise, ignore this move
                                	}
                                	else //in main LfD program
                                	{
	                                	previousPosition = "started";
	                                	started = true;
	                                	Console.WriteLine("Participant stuck out both arms to signify the beginning of interaction");
	                                	Console.WriteLine("***STARTING FIRST ITERATION OF DANCE***");
	                                	theHokeyPokeyDance.MoveNext(DanceMove.BHO);
	                                	Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

	                                	file.WriteLine("***Participant initiated interaction. Time: " + getTimeStamp(DateTime.Now));
	                                	file.WriteLine("\tCurrent dance state = " + theHokeyPokeyDance.CurrentState);

	                                    /*stateCount = STAGE1MANYMISTAKES;
	                                	//stage = "one";

	                                	//make sure the allMoves array is clear
	                                	for(int i=0; i<allMoves.Length; i++)
	                                	{
	                                		allMoves[i] = DONTMAKEERROR;
	                                	}

	                                	//numMistakes = random.Next(0,5) + 8; //pick randomly between 8-12 mistakes
	                                	numMistakes = 11;

	                                	//determine when the robot will make a mistake
	                                	int filler = 0;
	                                	while(filler < numMistakes)
	                                	{
	                                		randomNum = random.Next(0,20);
	                                		if(allMoves[randomNum] == DONTMAKEERROR)
	                                		{
	                                			//but don't make an error on a retraction ([x-limb out])
	                               				if(!(randomNum == 1 || randomNum == 6 || randomNum == 11 || randomNum == 16))
	                               				{
	                               					allMoves[randomNum] = MAKEERROR; //MAKEERROR = 1
	                               					filler++;
	                               				}
	                                		}
	                                	}

	                                    file.WriteLine("***MOVES ARRAY (0 = no mistake, 1 = mistake)***");
	                                    for(int i = 0; i < 20; i++)
	                                    {
	                                        if (i == 0)
	                                            file.Write("[" + allMoves[i] + ",");
	                                        else if (i == 19)
	                                            file.WriteLine(allMoves[i] + "]");
	                                        else
	                                            file.Write(allMoves[i] + ",");
	                                    }
	                                    file.WriteLine("***END MOVES ARRAY***");

	                                    //dataToSend = "getStarted 0 go";*/
                                        dataToSend = "getStarted";
	                                }
                                    readyToWrite = true;
                                }
                                else if(bothHandsIn() && (started || tutorialStarted))
                                {
                                	/*//this will be the signal I use to allow participants to terminate the tutorial program at any time
                                	if(inTutorial)
                                	{
                                		if(previousPosition.Equals("bothHandsIn"))
                                		{
                                			Console.WriteLine("Participant stuck both hands forward twice in a row. Terminating tutorial program");
                                			previousPosition = "bothHandsIn";

                                			file.WriteLine("Participant ended the tutorial program prematurely. Time: " + getTimeStamp(DateTime.Now));
                                			//file.WriteLine("\tCurrent state " + theHokeyPokeyDance.CurrentState);

                                            dataToSend = "end tutorial program";
                                		}
                                		else
                                		{
	                                		Console.WriteLine("Participant stuck both hands forward. Invalid move. If done again, tutorial program will end");
	                                		theHokeyPokeyDance.MoveNext(DanceMove.INVALID);
	                                		Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
		                                	previousPosition = "bothHandsIn";

		                                	file.WriteLine("Participant stuck both hands forward, an invalid move. Time: " + getTimeStamp(DateTime.Now));
		                                	file.WriteLine("\tCurrent state " + theHokeyPokeyDance.CurrentState);

                                            dataToSend = "warning next finish";
		                                }
                                	}
                                	else
                                	{
	                                	Console.WriteLine("Participant stuck both hands forward. Invalid move");
	                                	theHokeyPokeyDance.MoveNext(DanceMove.INVALID);
	                                	Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
	                                	previousPosition = "bothHandsIn";


	                                	file.WriteLine("Participant stuck both hands forward, an invalid move. Time: " + getTimeStamp(DateTime.Now));
	                                	file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

	                                	dataToSend = "nothing here alright";
                                	}*/
                                    if(waitingToConfirm)
                                    {
                                        inIncorrectLimbQuery = true;
                                        waitingToConfirm = false;
                                        dataToSend = "wrongconfirmation";
                                    }
                                    else
                                    {
                                        dataToSend = "nothing here alright";
                                    }

                                	readyToWrite = true;
                                }
                                else if(doingHokeyPokey() && (started || tutorialStarted))
                                {
                                	Console.WriteLine("Participant did the hokey pokey");
                                	//theHokeyPokeyDance.MoveNext(DanceMove.HP);
                                	//Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState); 

                                	file.WriteLine("Participant did the hokey pokey. Time: " + getTimeStamp(DateTime.Now));
                                	//file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                	
                                    if(waitingToConfirm)
                                    {
                                        inNextLimbQuery = true;
                                        waitingToConfirm = false;
                                        dataToSend = "rightconfirmation";
                                    }
                                	else if(inTutorial) //in tutorial
                                	{
                                		if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_HAND_SHAKE_5 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_HAND_SHAKE_10 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_LEG_SHAKE_15 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_LEG_SHAKE_20)
                                		{
                                			Console.WriteLine("Participant correctly did the hokey pokey");
                                			file.WriteLine("Participant correctly did the hokey pokey. Time: " + getTimeStamp(DateTime.Now));

                                			theHokeyPokeyDance.MoveNext(DanceMove.HP);
                                			Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                			file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                			previousPosition = "hokeyPokey";

                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.HOKEY_POKEY_21)
                                                dataToSend = "end tutorial 0";
                                            else
                                				dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                		}
                                		else //incorrect move in tutorial
                                		{
                                			Console.WriteLine("Participant incorrectly did the hokey pokey");
                                			file.WriteLine("Participant incorrectly did the hokey pokey. Time: " + getTimeStamp(DateTime.Now));

                                			Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                			file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                			dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                		}	
                                   	}
                                    else //non tutorial code
                                    { 
                                        //first check if it's a repeated movement
                                        if (previousPosition.Equals("hokeyPokey"))
                                        {
                                            //dataToSend = "repeated";
                                            dataToSend = "nothing here alright"; //do nothing for repeated movements
                                        }
                                        else
                                        {
                                            //moveSequence += " hokeypokey";
                                            if (activeLimb == "leftArm")
                                            {
                                                leftArmMovements[leftArmCounter] = "hokeypokey";

                                                //reset the counter since this is the last movement
                                                leftArmCounter = 0;
                                            }
                                            else if (activeLimb == "rightArm")
                                            {
                                                rightArmMovements[leftArmCounter] = "hokeypokey";

                                                //reset the counter since this is the last movement
                                                rightArmCounter = 0;
                                            }
                                            else if (activeLimb == "leftLeg")
                                            {
                                                leftLegMovements[leftArmCounter] = "hokeypokey";

                                                //reset the counter since this is the last movement
                                                leftLegCounter = 0;
                                            }
                                            else if (activeLimb == "rightLeg")
                                            {
                                                rightLegMovements[leftArmCounter] = "hokeypokey";

                                                //reset the counter since this is the last movement
                                                rightLegCounter = 0;
                                            }

                                            //set hokeypokeyDone flag
                                            hokeypokeyDone = true;
                                            dataToSend = "sequence";
                                        }
                                    }

                                	readyToWrite = true;                          	
                                }
                                else if(leftLegIn() && (started || tutorialStarted) && !waitingToConfirm)
                                {
                                    //new if statement
                                    if(inIncorrectLimbQuery) //person putting left leg in to tell the robot it got the limb wrong during the dance
                                    {
                                        activeLimb = "leftLeg";
                                        //clear the movement vector for that limb
                                        for(int p=0; p<100; p++)
                                        {
                                            leftLegMovements[p] = "null";
                                        }

                                        inIncorrectLimbQuery = false;
                                        dataToSend = "repeatLimb";
                                    }
                                    else if(inNextLimbQuery) //person putting their left leg in to tell the robot that it will now be learning this limb
                                    {
                                        activeLimb = "leftLeg";

                                        //check to make sure this limb hasn't already been done yet
                                        limbDone = false;
                                        for(int q=0; q<4; q++)
                                        {
                                            if(limbOrder[q].Equals("leftLeg"))
                                            {
                                                limbDone = true;
                                                break;
                                            }
                                        }

                                        if(!limbDone) //if the limb hasn't already been done, then add it to the limbOrder array
                                        {
                                            limbOrder[limbCounter] = "leftLeg";
                                            limbCounter++;
                                        }
                                        else //if the limb has already been done, then clear existing movements
                                        {
                                            for(int p=0; p<100; p++)
                                            {
                                                leftLegMovements[p] = "null";
                                            }
                                        }

                                        inNextLimbQuery = false;
                                        dataToSend = "nextLimb";
                                    }
                                	else if(previousPosition.Equals("leftLegIn") || previousPosition.Equals("leftLegShake"))
                                	{
                                		if(leftLegShaking(leftFootLastX))
                                		{
                                			skipLogic = true; //then don't process the following if block

                                			Console.WriteLine("Participant shook their left leg");
                                			file.WriteLine("Participant shook their left leg. Time: " + getTimeStamp(DateTime.Now));

                                            if(inTutorial) //in tutorial
                                            {
                                                if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_LEG_IN_14)
                                                {
                                                    Console.WriteLine("Participant correctly did left leg shake");
                                                    file.WriteLine("Participant correctly did left leg shake. Time: " + getTimeStamp(DateTime.Now));

                                                    //then the participant did the correct thing so update the dance state
                                                    theHokeyPokeyDance.MoveNext(DanceMove.LLS);
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    leftFootLastX = Convert.ToDouble(jointNamesAndData[15,2]);
                                                    previousPosition = "leftLegShake";

                                                    //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                    dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                                else //incorrect move in tutorial
                                                {
                                                    Console.WriteLine("Participant incorrectly performed left leg shake");
                                                    file.WriteLine("Participant incorrectly performed left leg shake. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    leftFootLastX = Convert.ToDouble(jointNamesAndData[15,2]);

                                                    //note that I did not update the previous position since the tutorial is a prompt

                                                    dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                			}
                                            else //TODO: add code for when it's not in tutorial here for left leg shake
                                            {	
                                                //first check to make sure the active limb is left leg
                                                if(!activeLimb.Equals("leftLeg"))
                                                {
                                                    dataToSend = "humanError";
                                                }
                                                else if(previousPosition.Equals("leftLegShake")) //check to see if its a repeated movement
                                                {
                                                    //then send a repeated movement notification to the robot
                                                    //dataToSend = "repeated";
                                                    dataToSend = "nothing here alright";
                                                }
                                                else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                                {
                                                    //add this demonstration to the list
                                                    leftLegMovements[leftLegCounter] = "leftLegShake";
                                                    leftLegCounter++;

                                                    dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                    		    }
                                            }
                                        }
                                	}
                                	
                                	if(!skipLogic) //for just left leg in (no shake)
                                	{
                                		Console.WriteLine("Participant put their left leg in");	
                                		file.WriteLine("Participant put their left leg in. Time: " + getTimeStamp(DateTime.Now));
                                		
                                		if(inTutorial) //in tutorial program
                                		{
                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.HOKEY_POKEY_11 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_LEG_OUT_13)
                                            {
                                                Console.WriteLine("Participant correctly put their left leg in");
                                                file.WriteLine("Participant correctly put their left leg in. Time: " + getTimeStamp(DateTime.Now));

                                                //then the participant did the correct thing so update the dance state
                                                theHokeyPokeyDance.MoveNext(DanceMove.LLI);
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                leftFootLastX = Convert.ToDouble(jointNamesAndData[15,2]);
                                                previousPosition = "leftLegIn";

                                                //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else //incorrect move
                                            {
                                                Console.WriteLine("Participant incorrectly put their left leg in");
                                                file.WriteLine("Participant incorrectly put their left leg in. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                leftFootLastX = Convert.ToDouble(jointNamesAndData[15,2]);

                                                //note that I did not update the previous position since the tutorial is a prompt

                                                dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                		}
                                        else //TODO: Add code for when it's not in the tutorial for left leg in here
                                        {
                                            //first check to make sure the active limb is left leg
                                            if(!activeLimb.Equals("leftLeg"))
                                            {
                                                dataToSend = "humanError";
                                            }
                                            else if(previousPosition.Equals("leftLegIn")) //check to see if its a repeated movement
                                            {
                                                //then send a repeated movement notification to the robot
                                                //dataToSend = "repeated";
                                                dataToSend = "nothing here alright";
                                            }
                                            else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                            {
                                                //add this demonstration to the list
                                                leftLegMovements[leftLegCounter] = "leftLegIn";
                                                leftLegCounter++;

                                                dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                            }
                                        }
                                	}

                                	skipLogic = false;
                                	readyToWrite = true;
                                }
                                else if(rightLegIn() && (started || tutorialStarted) && !waitingToConfirm)
                                {
                                	//TODO: Copy and modify from the above logic for left leg
                                    //new if statement
                                    if(inIncorrectLimbQuery) //person putting left leg in to tell the robot it got the limb wrong during the dance
                                    {
                                        activeLimb = "rightLeg";
                                        //clear the movement vector for that limb
                                        for(int p=0; p<100; p++)
                                        {
                                            rightLegMovements[p] = "null";
                                        }

                                        inIncorrectLimbQuery = false;
                                        dataToSend = "repeatLimb";
                                    }
                                    else if(inNextLimbQuery) //person putting their left leg in to tell the robot that it will now be learning this limb
                                    {
                                        activeLimb = "rightLeg";

                                        //check to make sure this limb hasn't already been done yet
                                        limbDone = false;
                                        for(int q=0; q<4; q++)
                                        {
                                            if(limbOrder[q].Equals("rightLeg"))
                                            {
                                                limbDone = true;
                                                break;
                                            }
                                        }

                                        if(!limbDone) //if the limb hasn't already been done, then add it to the limbOrder array
                                        {
                                            limbOrder[limbCounter] = "rightLeg";
                                            limbCounter++;
                                        }
                                        else //if the limb has already been done, then clear existing movements
                                        {
                                            for(int p=0; p<100; p++)
                                            {
                                                rightLegMovements[p] = "null";
                                            }
                                        }

                                        inNextLimbQuery = false;
                                        dataToSend = "nextLimb";
                                    }
                                    else if(previousPosition.Equals("rightLegIn") || previousPosition.Equals("rightLegShake"))
                                    {
                                        if(rightLegShaking(rightFootLastX))
                                        {
                                            skipLogic = true; //then don't process the following if block

                                            Console.WriteLine("Participant shook their right leg");
                                            file.WriteLine("Participant shook their right leg. Time: " + getTimeStamp(DateTime.Now));

                                            if(inTutorial) //in tutorial
                                            {
                                                if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_LEG_IN_19) 
                                                {
                                                    Console.WriteLine("Participant correctly did right leg shake");
                                                    file.WriteLine("Participant correctly did right leg shake. Time: " + getTimeStamp(DateTime.Now));

                                                    //then the participant did the correct thing so update the dance state
                                                    theHokeyPokeyDance.MoveNext(DanceMove.RLS);
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    rightFootLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                    previousPosition = "rightLegShake";

                                                    //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                    dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                                else //incorrect move in tutorial
                                                {
                                                    Console.WriteLine("Participant incorrectly performed right leg shake");
                                                    file.WriteLine("Participant incorrectly performed right leg shake. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    rightFootLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                    //note that I did not update the previous position since the tutorial is a prompt

                                                    dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                            } 
                                            else //TODO: add code for when it's not in tutorial here for left leg shake
                                            {  
                                                //first check to make sure the active limb is left leg
                                                if(!activeLimb.Equals("rightLeg"))
                                                {
                                                    dataToSend = "humanError";
                                                }
                                                else if(previousPosition.Equals("rightLegShake")) //check to see if its a repeated movement
                                                {
                                                   //then send a repeated movement notification to the robot
                                                   //dataToSend = "repeated";
                                                   dataToSend = "nothing here alright";
                                                }
                                                else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                                {
                                                    //add this demonstration to the list
                                                    rightLegMovements[rightLegCounter] = "rightLegShake";
                                                    rightLegCounter++;

                                                    dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                                }
                                            }
                                        }
                                    }
                                    
                                    if(!skipLogic) //for just right leg in (no shake)
                                    {
                                        Console.WriteLine("Participant put their right leg in"); 
                                        file.WriteLine("Participant put their right leg in. Time: " + getTimeStamp(DateTime.Now));
                                        
                                        if(inTutorial) //in tutorial program
                                        {
                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.HOKEY_POKEY_16 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_LEG_OUT_18)
                                            {
                                                Console.WriteLine("Participant correctly put their right leg in");
                                                file.WriteLine("Participant correctly put their right leg in. Time: " + getTimeStamp(DateTime.Now));

                                                //then the participant did the correct thing so update the dance state
                                                theHokeyPokeyDance.MoveNext(DanceMove.RLI);
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                rightFootLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                previousPosition = "rightLegIn";

                                                //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else //incorrect move
                                            {
                                                Console.WriteLine("Participant incorrectly put their right leg in");
                                                file.WriteLine("Participant incorrectly put their right leg in. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                rightFootLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                //note that I did not update the previous position since the tutorial is a prompt

                                                dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                        }
                                        else //TODO: Add code for when it's not in the tutorial for left leg in here
                                        {
                                            //first check to make sure the active limb is left leg
                                            if(!activeLimb.Equals("rightLeg"))
                                            {
                                                dataToSend = "humanError";
                                            }
                                            else if(previousPosition.Equals("rightLegIn")) //check to see if its a repeated movement
                                            {
                                                //then send a repeated movement notification to the robot
                                                //dataToSend = "repeated";
                                                dataToSend = "nothing here alright";
                                            }
                                            else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                            {
                                                //add this demonstration to the list
                                                rightLegMovements[rightLegCounter] = "rightLegIn";
                                                rightLegCounter++;

                                                dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                            }
                                        }
                                    }

                                    skipLogic = false;
                                    readyToWrite = true;
                                }
                                else if(leftArmIn() && (started || tutorialStarted) && !waitingToConfirm)
                                {
                                    //TODO: Copy and modify from the above logic for left leg
                                    //new if statement
                                    if(inIncorrectLimbQuery) //person putting left leg in to tell the robot it got the limb wrong during the dance
                                    {
                                        activeLimb = "leftArm";
                                        //clear the movement vector for that limb
                                        for(int p=0; p<100; p++)
                                        {
                                            leftArmMovements[p] = "null";
                                        }

                                        inIncorrectLimbQuery = false;
                                        dataToSend = "repeatLimb";
                                    }
                                    else if(inNextLimbQuery) //person putting their left leg in to tell the robot that it will now be learning this limb
                                    {
                                        activeLimb = "leftArm";

                                        //check to make sure this limb hasn't already been done yet
                                        limbDone = false;
                                        for(int q=0; q<4; q++)
                                        {
                                            if(limbOrder[q].Equals("leftArm"))
                                            {
                                                limbDone = true;
                                                break;
                                            }
                                        }

                                        if(!limbDone) //if the limb hasn't already been done, then add it to the limbOrder array
                                        {
                                            limbOrder[limbCounter] = "leftArm";
                                            limbCounter++;
                                        }
                                        else //if the limb has already been done, then clear existing movements
                                        {
                                            for(int p=0; p<100; p++)
                                            {
                                                leftArmMovements[p] = "null";
                                            }
                                        }

                                        inNextLimbQuery = false;
                                        dataToSend = "nextLimb";
                                    }
                                    else if(previousPosition.Equals("leftArmIn") || previousPosition.Equals("leftArmShake"))
                                    {
                                        if(leftArmShaking(leftHandLastX))
                                        {
                                            skipLogic = true; //then don't process the following if block

                                            Console.WriteLine("Participant shook their left arm");
                                            file.WriteLine("Participant shook their left arm. Time: " + getTimeStamp(DateTime.Now));

                                            if(inTutorial) //in tutorial
                                            {
                                                if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_HAND_IN_4) //CHANGE
                                                {
                                                    Console.WriteLine("Participant correctly did left arm shake");
                                                    file.WriteLine("Participant correctly did left arm shake. Time: " + getTimeStamp(DateTime.Now));

                                                    //then the participant did the correct thing so update the dance state
                                                    theHokeyPokeyDance.MoveNext(DanceMove.LHS);
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    leftHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                    previousPosition = "leftArmShake";

                                                    //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                    dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                                else //incorrect move in tutorial
                                                {
                                                    Console.WriteLine("Participant incorrectly performed left arm shake");
                                                    file.WriteLine("Participant incorrectly performed left arm shake. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    leftHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                    //note that I did not update the previous position since the tutorial is a prompt

                                                    dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                            }   
                                            else //TODO: add code for when it's not in tutorial here for left leg shake
                                            {
                                                //first check to make sure the active limb is left leg
                                                if(!activeLimb.Equals("leftArm"))
                                                {
                                                    dataToSend = "humanError";
                                                }
                                                else if(previousPosition.Equals("leftArmShake")) //check to see if its a repeated movement
                                                {
                                                   //then send a repeated movement notification to the robot
                                                   //dataToSend = "repeated";
                                                   dataToSend = "nothing here alright";
                                                }
                                                else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                                {
                                                    //add this demonstration to the list
                                                    leftArmMovements[rightLegCounter] = "leftArmShake";
                                                    leftArmCounter++;

                                                    dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                                }
                                            }
                                        }
                                    }
                                    
                                    if(!skipLogic) //for just left arm in (no shake)
                                    {
                                        Console.WriteLine("Participant put their left arm in"); 
                                        file.WriteLine("Participant put their left arm in. Time: " + getTimeStamp(DateTime.Now));
                                        
                                        if(inTutorial) //in tutorial program
                                        {
                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.DEFAULT_STATE_22 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_HAND_OUT_3)
                                            {
                                                Console.WriteLine("Participant correctly put their left arm in");
                                                file.WriteLine("Participant correctly put their left arm in. Time: " + getTimeStamp(DateTime.Now));

                                                //then the participant did the correct thing so update the dance state
                                                theHokeyPokeyDance.MoveNext(DanceMove.LHI);
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                leftHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                previousPosition = "leftArmIn";

                                                //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else //incorrect move
                                            {
                                                Console.WriteLine("Participant incorrectly put their left arm in");
                                                file.WriteLine("Participant incorrectly put their left arm in. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                leftHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                //note that I did not update the previous position since the tutorial is a prompt

                                                dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                        }
                                        else //TODO: Add code for when it's not in the tutorial for left leg in here
                                        {
                                            //first check to make sure the active limb is left leg
                                            if(!activeLimb.Equals("leftArm"))
                                            {
                                                dataToSend = "humanError";
                                            }
                                            else if(previousPosition.Equals("leftArmIn")) //check to see if its a repeated movement
                                            {
                                                //then send a repeated movement notification to the robot
                                                //dataToSend = "repeated";
                                                dataToSend = "nothing here alright";
                                            }
                                            else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                            {
                                                //add this demonstration to the list
                                                leftArmMovements[leftArmCounter] = "leftArmIn";
                                                leftArmCounter++;

                                                dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                            }
                                        }
                                    }

                                    skipLogic = false;
                                    readyToWrite = true;
                                }
                                else if(rightArmIn() && (started || tutorialStarted) && !waitingToConfirm)
                                {
                                	//TODO: Copy and modify from the above logic for left leg
                                    //new if statement
                                    if(inIncorrectLimbQuery) //person putting left leg in to tell the robot it got the limb wrong during the dance
                                    {
                                        activeLimb = "rightArm";
                                        //clear the movement vector for that limb
                                        for(int p=0; p<100; p++)
                                        {
                                            rightArmMovements[p] = "null";
                                        }

                                        inIncorrectLimbQuery = false;
                                        dataToSend = "repeatLimb";
                                    }
                                    else if(inNextLimbQuery) //person putting their left leg in to tell the robot that it will now be learning this limb
                                    {
                                        activeLimb = "rightArm";

                                        //check to make sure this limb hasn't already been done yet
                                        limbDone = false;
                                        for(int q=0; q<4; q++)
                                        {
                                            if(limbOrder[q].Equals("rightArm"))
                                            {
                                                limbDone = true;
                                                break;
                                            }
                                        }

                                        if(!limbDone) //if the limb hasn't already been done, then add it to the limbOrder array
                                        {
                                            limbOrder[limbCounter] = "rightArm";
                                            limbCounter++;
                                        }
                                        else //if the limb has already been done, then clear existing movements
                                        {
                                            for(int p=0; p<100; p++)
                                            {
                                                rightArmMovements[p] = "null";
                                            }
                                        }

                                        inNextLimbQuery = false;
                                        dataToSend = "nextLimb";
                                    }
                                    else if(previousPosition.Equals("rightArmIn") || previousPosition.Equals("rightArmShake"))
                                    {
                                        if(rightArmShaking(rightHandLastX))
                                        {
                                            skipLogic = true; //then don't process the following if block

                                            Console.WriteLine("Participant shook their right arm");
                                            file.WriteLine("Participant shook their right arm. Time: " + getTimeStamp(DateTime.Now));

                                            if(inTutorial) //in tutorial
                                            {
                                                if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_HAND_IN_9) //CHANGE
                                                {
                                                    Console.WriteLine("Participant correctly did right arm shake");
                                                    file.WriteLine("Participant correctly did right arm shake. Time: " + getTimeStamp(DateTime.Now));

                                                    //then the participant did the correct thing so update the dance state
                                                    theHokeyPokeyDance.MoveNext(DanceMove.RHS);
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    rightHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                    previousPosition = "rightArmShake";

                                                    //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                    dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                                else //incorrect move in tutorial
                                                {
                                                    Console.WriteLine("Participant incorrectly performed right arm shake");
                                                    file.WriteLine("Participant incorrectly performed right arm shake. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                    Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                    file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                    rightHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                    //note that I did not update the previous position since the tutorial is a prompt

                                                    dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                }
                                            }   
                                            else //TODO: add code for when it's not in tutorial here for left leg shake
                                            {
                                                //first check to make sure the active limb is left leg
                                                if(!activeLimb.Equals("rightArm"))
                                                {
                                                    dataToSend = "humanError";
                                                }
                                                else if(previousPosition.Equals("rightArmShake")) //check to see if its a repeated movement
                                                {
                                                   //then send a repeated movement notification to the robot
                                                  // dataToSend = "repeated";
                                                  dataToSend = "nothing here alright";
                                                }
                                                else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                                {
                                                    //add this demonstration to the list
                                                    rightArmMovements[rightLegCounter] = "rightArmShake";
                                                    rightArmCounter++;

                                                    dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                                }
                                            }
                                        }
                                    }
                                    
                                    if(!skipLogic) //for just right arm in (no shake)
                                    {
                                        Console.WriteLine("Participant put their right arm in"); 
                                        file.WriteLine("Participant put their right arm in. Time: " + getTimeStamp(DateTime.Now));
                                        
                                        if(inTutorial) //in tutorial program
                                        {
                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.HOKEY_POKEY_6 || (int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_HAND_OUT_8)
                                            {
                                                Console.WriteLine("Participant correctly put their right arm in");
                                                file.WriteLine("Participant correctly put their right arm in. Time: " + getTimeStamp(DateTime.Now));

                                                //then the participant did the correct thing so update the dance state
                                                theHokeyPokeyDance.MoveNext(DanceMove.RHI);
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                rightHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE
                                                previousPosition = "rightArmIn";

                                                //dataToSend = "leftArmShakeCorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                                dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else //incorrect move
                                            {
                                                Console.WriteLine("Participant incorrectly put their right arm in");
                                                file.WriteLine("Participant incorrectly put their right arm in. Time: " + getTimeStamp(DateTime.Now));
                                                    
                                                Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                                file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                                rightHandLastX = Convert.ToDouble(jointNamesAndData[15,2]); //CHANGE

                                                //note that I did not update the previous position since the tutorial is a prompt

                                                dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                        }
                                        else //TODO: Add code for when it's not in the tutorial for left leg in here
                                        {
                                            //first check to make sure the active limb is left leg
                                            if(!activeLimb.Equals("rightArm"))
                                            {
                                                dataToSend = "humanError";
                                            }
                                            else if(previousPosition.Equals("rightArmIn")) //check to see if its a repeated movement
                                            {
                                                //then send a repeated movement notification to the robot
                                                //dataToSend = "repeated";
                                                dataToSend = "nothing here alright";
                                            }
                                            else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                            {
                                                //add this demonstration to the list
                                                rightArmMovements[leftArmCounter] = "rightArmIn";
                                                rightArmCounter++;

                                                dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                            }
                                        }
                                    }

                                    skipLogic = false;
                                    readyToWrite = true;
                                }
                                else if(inDefault() && (started || tutorialStarted) & !(waitingToConfirm || inIncorrectLimbQuery || inNextLimbQuery))
                                {
                                    if(!previousPosition.Equals("default") && !previousPosition.Equals("started")) //doing this to avoid multiple "Ok I got it" from the robot when the participant isn't doing anything
                                    {
                                        Console.WriteLine("Participant went to default position");
                                        file.WriteLine("Participant went to default position. Time: " + getTimeStamp(DateTime.Now));
                                        
                                        if(inTutorial) //in tutorial
                                        {
                                            if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_HAND_IN_2)
                                            {
                                            	Console.WriteLine("Participant correctly put their left hand out");
                                				file.WriteLine("Participant correctly put their left hand out. Time: " + getTimeStamp(DateTime.Now));

                                				theHokeyPokeyDance.MoveNext(DanceMove.DEFAULT);
                                				Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                				file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                				previousPosition = "default";

                                				dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_HAND_IN_7)
                                            {
                                            	Console.WriteLine("Participant correctly put their right hand out");
                                				file.WriteLine("Participant correctly put their right hand out. Time: " + getTimeStamp(DateTime.Now));

                                				theHokeyPokeyDance.MoveNext(DanceMove.DEFAULT);
                                				Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                				file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                				previousPosition = "default";

                                				dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.LEFT_LEG_IN_12)
                                            {
                                            	Console.WriteLine("Participant correctly put their left leg out");
                                				file.WriteLine("Participant correctly put their left leg out. Time: " + getTimeStamp(DateTime.Now));

                                				theHokeyPokeyDance.MoveNext(DanceMove.DEFAULT);
                                				Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                				file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                				previousPosition = "default";

                                				dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else if((int)theHokeyPokeyDance.CurrentState == (int)DanceState.RIGHT_LEG_IN_17)
                                            {
                                            	Console.WriteLine("Participant correctly put their right leg out");
                                				file.WriteLine("Participant correctly put their right leg out. Time: " + getTimeStamp(DateTime.Now));

                                				theHokeyPokeyDance.MoveNext(DanceMove.DEFAULT);
                                				Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                				file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                				previousPosition = "default";

                                				dataToSend = "correct " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                            else //incorrect move
                                            {
                                            	Console.WriteLine("Participant incorrectly put their hand/leg out and returned to default standing position");
                                				file.WriteLine("Participant incorrectly put their hand/leg out and returned to default standing position. Time: " + getTimeStamp(DateTime.Now));
                                            	
                                				Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                				file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);

                                				dataToSend = "incorrect " + theHokeyPokeyDance.CurrentState + " yes";
                                            }
                                        }
                                        else //TODO: Add code for when it's not in tutorial for default position here
                                        {
                                            if(previousPosition.Equals("default")) //check to see if its a repeated movement
                                            {
                                                //then send a repeated movement notification to the robot
                                                //dataToSend = "repeated";
                                                dataToSend = "nothing here alright";
                                            }
                                            else //if the participant is using the correct limb and it's not a repeated movement, everything is good
                                            {
                                                //add this demonstration to the list
                                                moveSequence += " default";

                                                dataToSend = "actionPerformed"; //I'll actually send this so that the robot can respond "okay" after each move
                                            }
                                        }
                                    }
                                    else //participant going to default after the previous position was default, so ignore
                                    {
                                        dataToSend = "nothing here alright";
                                    }
                                	
                                	readyToWrite = true;
                                }
                                else //participant did some movement that was not accounted for **OR** the interaction hasn't started yet
                                {
                                    if(started || tutorialStarted) //participant did some invalid move after the interaction started
                                    {
                                        if(!previousPosition.Equals("invalid"))
                                        {
                                        	invalidCounter++;

                                        	if(invalidCounter > 30)
                                        	{
                                        		invalidCounter = 0;

                                            	Console.WriteLine("Participant did some invalid move");
                                            	theHokeyPokeyDance.MoveNext(DanceMove.INVALID);
                                            	Console.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                            	previousPosition = "invalid";

                                            	file.WriteLine("Participant did some invalid move");
                                            	file.WriteLine("\tCurrent state = " + theHokeyPokeyDance.CurrentState);
                                            	file.Flush();	
                                        	}
                                        	dataToSend = "nothing here alright";
                                        }

                                    }
                                    else //this represents the participant doing whatever before the interaction begins
                                    {
                                    	counter++;

                                    	if(counter > 60 && !welcomeFlag) //guessing the the Kinect captures at roughly 15fps so wait 4 seconds
 										{	
 											welcomeFlag = true;
 											dataToSend = "bodyDetected 0 go";
 										}
 										else
                                        	dataToSend = "nothing here alright";
                                    }

                                   	readyToWrite = true;
                                }
                            } //end of "if(connected && !dataBeingPrepped" loop
                        } //end of "if (body.IsTracked)" loop
                    } //end of foreach frame loop
                }
                //increment frame number
            	++this.frameNumber;
        	}
        }

		/* Function Name:   mapJointNamesToOpenNI
         * Parameters:      none
         * Returns:         string of comma separated joints
         * Purpose:         Convert the Enum names of Kinect SDK to the names defined by OpenNI skeleton tracker.  Also
         *                  renames joints not defined by the OpenNI tracker to have similar form.
         */
        private string mapJointNamesToOpenNI()
        {
            string namesString = null;
            string[] enumNames = Enum.GetNames(typeof(JointType));
            for (int i = 0; i < enumNames.Length; i++)
            {
                namesString += enumNames[i] + ","; 
            }
            namesString = namesString.TrimEnd(',');
            return namesString;
        }

        /* Function Name:   getTicksSinceEpoch
         * Parameters:      currentFrameTime - RelativeTime property of the current frame; firstFrameTime - RelativeTime property 
         *                  of the first frame received (recorded separately for each type of stream)
         * Returns:         long, number of ticks since the epoch
         * Purpose:         Calculate the number of ticks (1 tick = 100 ns) since the beginning of Unix time (January 1, 1970, 
         *                  00:00:00.000).  This calculation will vary depending on if a KStudio recording provides the data, 
         *                  or if it is a live data stream.
         */
        private long getTicksSinceEpoch(long currentFrameTime, long firstFrameTime)
        {
            long timeSinceStart = currentFrameTime - firstFrameTime;
            long ticksSinceEpoch = this.startTime.Ticks + timeSinceStart - (new DateTime(1970, 1, 1, 0, 0, 0)).Ticks;
            return ticksSinceEpoch;
        }

        /* Function Name:   calculateFPS
         * Parameters:      none
         * Returns:         void
         * Purpose:         Calculates FPS of multisource frames received and writes updated data to the console each second.
         */
        private void calculateFPS()
        {
            this.framesSinceUpdate++;
            if (DateTime.Now >= this.nextStatusUpdate)
            {
                if (this.frameTime.IsRunning)
                {
                    this.frameTime.Stop();
                    double fps = this.framesSinceUpdate / this.frameTime.Elapsed.TotalSeconds;
                    //Console.WriteLine("FPS: " + fps); //Cory's note, I need to uncomment this later
                    this.nextStatusUpdate = DateTime.Now + TimeSpan.FromSeconds(1);
                    this.frameTime.Reset();
                }
                if (!this.frameTime.IsRunning)
                {
                    this.framesSinceUpdate = 0;
                    this.frameTime.Start();
                }
            }
        }

        private string createMovementSequence()
        {
            string tempMovementSequence = "";
            int r=0;
            while(!limbOrder[r].Equals("null"))
            {
                if(limbOrder[r].Equals("leftArm"))
                {
                    int s=0;
                    while(!leftArmMovements[s].Equals("null"))
                    {
                        tempMovementSequence += leftArmMovements[s] + " ";
                        s++;
                    }
                }
                else if(limbOrder[r].Equals("rightArm"))
                {
                    int s=0;
                    while(!rightArmMovements[s].Equals("null"))
                    {
                        tempMovementSequence += rightArmMovements[s] + " ";
                        s++;
                    }
                }
                else if(limbOrder[r].Equals("leftLeg"))
                {
                    int s=0;
                    while(!leftLegMovements[s].Equals("null"))
                    {
                        tempMovementSequence += leftLegMovements[s] + " ";
                        s++;
                    }
                }
                else if(limbOrder[r].Equals("rightLeg"))
                {
                    int s=0;
                    while(!leftArmMovements[s].Equals("null"))
                    {
                        tempMovementSequence += rightLegMovements[s] + " ";
                        s++;
                    }
                }

                r++;
            }

            return tempMovementSequence;
        }

        private bool leftArmIn()
        {
        	//do the same kind of check for their left hand
            double zRightShoulderLeftHand = (Convert.ToDouble(jointNamesAndData[8,4])) - (Convert.ToDouble(jointNamesAndData[7,4]));
            
            if(zRightShoulderLeftHand > 0.4)
                return true;
            else
            	return false;
        }

        private bool leftArmShaking(double temp)
        {
        	double xLeftHand = Convert.ToDouble(jointNamesAndData[7,2]);
        	
        	if((xLeftHand - temp) > 0.17 || (xLeftHand - temp) < -0.17)
        		return true;
        	else
        		return false;
        }
        
        private bool rightArmIn()
        {
        	//check to see if the person is holding out their right hand
            //need to see the difference in z dimension of right hand and left shoulder
            //z is the 5th value
            double zLeftShoulderRightHand = (Convert.ToDouble(jointNamesAndData[4,4])) - (Convert.ToDouble(jointNamesAndData[11,4]));
            
           	if(zLeftShoulderRightHand > 0.4)
                return true;
            else
            	return false;
        }
        
        private bool rightArmShaking(double temp)
        {
        	double xRightHand = Convert.ToDouble(jointNamesAndData[11,2]);
        	
        	if((xRightHand - temp) > 0.17 || (xRightHand - temp) < -0.17)
        		return true;
        	else
        		return false; 
        }
        
        private bool bothHandsIn()
        {
        	return (leftArmIn() && rightArmIn());
        }

        private bool bothHandsOutToSide()
        {
            //hands need to be far apart but the hands should not be higher than the shoulders

            double xRightHand = Convert.ToDouble(jointNamesAndData[11,2]);
            double xLeftHand = Convert.ToDouble(jointNamesAndData[7,2]);
            double yRightHand = Convert.ToDouble(jointNamesAndData[11,3]);
            double yLeftHand = Convert.ToDouble(jointNamesAndData[7,3]);
            double yRightShoulder = Convert.ToDouble(jointNamesAndData[8,3]);
            double yLeftShoulder = Convert.ToDouble(jointNamesAndData[4,3]);

            double diffHands = xRightHand - xLeftHand;
            double diffLShoulderLHand = yLeftHand - yLeftShoulder;
            double diffRShoulderRHand = yRightHand - yRightShoulder;

            if (diffHands > 1.5 || diffHands < -1.5)
            {
                if ((diffLShoulderLHand < 0.1 && diffLShoulderLHand > -0.1) && (diffRShoulderRHand < 0.1 && diffRShoulderRHand > -0.1))
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
        
        private bool leftLegIn()
        {
        	//check to see if the person is holding out their left foot or right foot
            //need to see the difference in z dimension of left foot and right foot
                                                 
            double zRightFootLeftFoot = (Convert.ToDouble(jointNamesAndData[19,4])) - (Convert.ToDouble(jointNamesAndData[15,4]));
            
            if(zRightFootLeftFoot > 0.2) //if right foot is further away, then left foot is in
            	return true;
            else
        		return false;
        }

        private bool leftLegShaking(double temp)
        {
            double xLeftFoot = Convert.ToDouble(jointNamesAndData[15,2]);

            if((xLeftFoot - temp) > 0.1 || (xLeftFoot - temp) < -0.1)
                return true;
            else
                return false;
        }
        
        private bool rightLegIn()
        {
        	double zLeftFootRightFoot = (Convert.ToDouble(jointNamesAndData[15,4])) - (Convert.ToDouble(jointNamesAndData[19,4]));
        	
        	if(zLeftFootRightFoot > 0.2) //if the left foot is further away, then right foot is in
           		return true;
            else
        		return false;
        }

        private bool rightLegShaking(double temp)
        {
            double xRightFoot = Convert.ToDouble(jointNamesAndData[19,2]);

            if ((xRightFoot - temp) > 0.1 || (xRightFoot - temp) < -0.1)
                return true;
            else
                return false;
        }

        private bool inDefault()
        {
            //check if person is in default position (both hands down)
            //need to see difference in y dimension of left hand and right shoulder
            //also need to see difference in y dimension of right hand and right shoulder
            // y is the 4th value in the array
                                
            double yLeftShoulderLeftHand = (Convert.ToDouble(jointNamesAndData[4,3])) - (Convert.ToDouble(jointNamesAndData[7,3]));
            double yRightShoulderRightHand = (Convert.ToDouble(jointNamesAndData[8,3])) - (Convert.ToDouble(jointNamesAndData[11,3]));
            
            if((yLeftShoulderLeftHand > 0.35) && (yRightShoulderRightHand > 0.35)) //shoulder should be higher (greater y value)  
            {
                if(!leftLegIn() && !rightLegIn()) //this also covers the shaking
                    return true;
                else
                    return false;
            } 
            else
                return false;
        }
        
        private bool doingHokeyPokey()
        {
        	//hands above head
        	double yLeftHandLeftShoulder = (Convert.ToDouble(jointNamesAndData[7,3])) - (Convert.ToDouble(jointNamesAndData[4,3]));
            double yRightHandRightShoulder = (Convert.ToDouble(jointNamesAndData[11,3])) - (Convert.ToDouble(jointNamesAndData[8,3]));
            
            if(yLeftHandLeftShoulder > 0.3 && yRightHandRightShoulder > 0.3)
            	return true;
            else
            	return false;
        }
    }

	class ConsoleApp
	{
		/* Function Name:   Main
	     * Parameters:      args-command line arguments
	     * Returns:         void
	     * Purpose:         Entry point to application.  Instantiates a SavingKinectData object and runs until
	     *                  the user inputs ENTER.
	     */
	    static void Main(string[] args)
	    {
	        //instantiating SavingKinectData object sets up subscriptions to frame arrival events 
	        SavingKinectData prog = new SavingKinectData();

	        //wait for user input to end program
	        Console.ReadLine();

	        //close kinect and stop server
	        if(prog.sensor != null) prog.sensor.Close();
	        //prog.server.stopServer();
	    }
	}
}
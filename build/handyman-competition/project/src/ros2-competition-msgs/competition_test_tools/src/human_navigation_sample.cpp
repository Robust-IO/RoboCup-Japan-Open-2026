#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#include <human_navigation_msgs/msg/human_navi_object_info.hpp>
#include <human_navigation_msgs/msg/human_navi_destination.hpp>
#include <human_navigation_msgs/msg/human_navi_task_info.hpp>
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_avatar_status.hpp>
#include <human_navigation_msgs/msg/human_navi_object_status.hpp>

class HumanNavigationSample
{
private:
	enum Step
	{
		Initialize,
		Ready,
		WaitTaskInfo,
		GuideForTakingObject,
		GuideForPlacement,
		WaitTaskFinished,
		TaskFinished
	};

	enum class SpeechState
	{
		None,
		WaitingState,
		Speaking,
		Speakable
	};

	// human navigation message from/to the moderator
	const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
	const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
	const std::string MSG_TASK_FAILED        = "Task_failed";
	const std::string MSG_TASK_FINISHED      = "Task_finished";
	const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
	const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
	const std::string MSG_REQUEST            = "Guidance_request";
	const std::string MSG_SPEECH_STATE       = "Speech_state";
	const std::string MSG_SPEECH_RESULT      = "Speech_result";

	const std::string MSG_I_AM_READY        = "I_am_ready";
	const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
	const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
	const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

	// display type of guidance message panels for the avatar (test subject)
	const std::string DISPLAY_TYPE_ALL         = "All";
	const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
	const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
	const std::string DISPLAY_TYPE_NONE        = "None";

	int step;
	SpeechState speechState;

	bool isStarted;
	bool isFinished;

	bool isTaskInfoReceived;
	bool isRequestReceived;

	rclcpp::Time timePrevSpeechStateConfirmed;

	bool isSentGetAvatarStatus;
	bool isSentGetObjectStatus;

	human_navigation_msgs::msg::HumanNaviTaskInfo taskInfo;
	std::string guideMsg;

	human_navigation_msgs::msg::HumanNaviAvatarStatus avatarStatus;
	human_navigation_msgs::msg::HumanNaviObjectStatus objectStatus;

	std::shared_ptr<rclcpp::Node> nodeHandle;

	void init()
	{
		step = Initialize;
		speechState = SpeechState::None;

		reset();
	}

	void reset()
	{
		isStarted             = false;
		isFinished            = false;
		isTaskInfoReceived    = false;
		isRequestReceived     = false;
		isSentGetAvatarStatus = false;
		isSentGetObjectStatus = false;
	}

	// send humanNaviMsg to the moderator (Unity)
	void sendMessage(rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr publisher, const std::string &message)
	{
		human_navigation_msgs::msg::HumanNaviMsg human_navi_msg;
		human_navi_msg.message = message;
		publisher->publish(human_navi_msg);

		RCLCPP_INFO(nodeHandle->get_logger(), "Send message:%s", message.c_str());
	}

	void sendGuidanceMessage(rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr publisher, const std::string &message, const std::string displayType)
	{
		human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
		guidanceMessage.message = message;
		guidanceMessage.display_type = displayType;
		guidanceMessage.source_language = ""; // Blank or ISO-639-1 language code, e.g. "en".
		guidanceMessage.target_language = ""; // Blank or ISO-639-1 language code, e.g. "ja".
		publisher->publish(guidanceMessage);

		speechState = SpeechState::Speaking;

		RCLCPP_INFO(nodeHandle->get_logger(), "Send guide message: %s : %s", guidanceMessage.message.c_str(), guidanceMessage.display_type.c_str());
	}

	// receive humanNaviMsg from the moderator (Unity)
	void messageCallback(const human_navigation_msgs::msg::HumanNaviMsg::ConstSharedPtr message)
	{
		RCLCPP_INFO(nodeHandle->get_logger(), "Subscribe message: %s : %s", message->message.c_str(), message->detail.c_str());

		if(message->message==MSG_ARE_YOU_READY)
		{
			isStarted = true;
		}
		else if(message->message==MSG_REQUEST)
		{
			if(isTaskInfoReceived && !isFinished)
			{
				isRequestReceived = true;
			}
		}
		else if(message->message==MSG_TASK_SUCCEEDED)
		{
		}
		else if(message->message==MSG_TASK_FAILED)
		{
		}
		else if(message->message==MSG_TASK_FINISHED)
		{
			isFinished = true;
		}
		else if(message->message==MSG_GO_TO_NEXT_SESSION)
		{
			RCLCPP_INFO(nodeHandle->get_logger(), "Go to next session");
			step = Initialize;
		}
		else if(message->message==MSG_MISSION_COMPLETE)
		{
			//exit(EXIT_SUCCESS);
		}
		else if(message->message==MSG_SPEECH_STATE)
		{
			if(message->detail=="Is_speaking")
			{
				speechState = SpeechState::Speaking;
			}
			else
			{
				speechState = SpeechState::Speakable;
			}
		}
		else if(message->message==MSG_SPEECH_RESULT)
		{
			RCLCPP_INFO(nodeHandle->get_logger(), "Speech result: %s", message->detail.c_str());
		}
	}

	// receive taskInfo from the moderator (Unity)
	void taskInfoMessageCallback(const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr message)
	{
		taskInfo = *message;

		RCLCPP_INFO_STREAM(
			nodeHandle->get_logger(),
			"Subscribe task info message:" << std::endl <<
			"Environment ID: " << taskInfo.environment_id << std::endl <<
			"Target object name: " << std::endl << human_navigation_msgs::msg::to_yaml(taskInfo.target_object) <<
			"Destination pos: " << std::endl << human_navigation_msgs::msg::to_yaml(taskInfo.destination)
		);

		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.non_target_objects[i]) << std::endl;
		}

		int numOfFurniture = taskInfo.furniture.size();
		std::cout << "Number of furniture: " << numOfFurniture << std::endl;
		std::cout << "Furniture objects:" << std::endl;
		for(int i=0; i<numOfFurniture; i++)
		{
			std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.furniture[i]) << std::endl;
		}

		isTaskInfoReceived = true;
	}

	void avatarStatusMessageCallback(const human_navigation_msgs::msg::HumanNaviAvatarStatus::ConstSharedPtr message)
	{
		avatarStatus = *message;

		RCLCPP_INFO_STREAM(
			nodeHandle->get_logger(),
			"Subscribe avatar status message:" << std::endl <<
			"Head: " << std::endl      << geometry_msgs::msg::to_yaml(avatarStatus.head) <<
			"LeftHand: " << std::endl  << geometry_msgs::msg::to_yaml(avatarStatus.left_hand) <<
			"rightHand: " << std::endl << geometry_msgs::msg::to_yaml(avatarStatus.right_hand) <<
			"objctInLeftHand: "        << avatarStatus.object_in_left_hand << std::endl <<
			"objectInRightHand: "      << avatarStatus.object_in_right_hand << std::endl <<
			"isTargetObjectInLeftHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_left_hand << std::endl <<
			"isTargetObjectInRightHand: " << std::boolalpha << (bool)avatarStatus.is_target_object_in_right_hand << std::endl
		);
		isSentGetAvatarStatus = false;
	}

	void objectStatusMessageCallback(const human_navigation_msgs::msg::HumanNaviObjectStatus::ConstSharedPtr message)
	{
		objectStatus = *message;

		RCLCPP_INFO_STREAM(
			nodeHandle->get_logger(),
			"Subscribe object status message:" << std::endl <<
			"Target object: " << std::endl << human_navigation_msgs::msg::to_yaml(taskInfo.target_object)
		);

		int numOfNonTargetObjects = taskInfo.non_target_objects.size();
		std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
		std::cout << "Non-target objects:" << std::endl;
		for(int i=0; i<numOfNonTargetObjects; i++)
		{
			std::cout << human_navigation_msgs::msg::to_yaml(taskInfo.non_target_objects[i]) << std::endl;
		}

		isSentGetObjectStatus = false;
	}

	bool speakGuidanceMessage(rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr pubHumanNaviMsg, rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pubGuidanceMsg, std::string message, int interval = 1)
	{
		if(speechState == SpeechState::Speakable)
		{
			sendGuidanceMessage(pubGuidanceMsg, message, DISPLAY_TYPE_ALL);
			speechState = SpeechState::None;
			return true;
		}
		else if(speechState == SpeechState::None || speechState == SpeechState::Speaking)
		{
			if(timePrevSpeechStateConfirmed.seconds() + interval < nodeHandle->now().seconds())
			{
				sendMessage(pubHumanNaviMsg, MSG_GET_SPEECH_STATE);
				timePrevSpeechStateConfirmed = nodeHandle->now();
				speechState = SpeechState::WaitingState;
			}
		}

		return false;
	}

public:
	int run([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
	{
		nodeHandle = std::make_shared<rclcpp::Node>("human_navi_sample");

		rclcpp::Rate loopRate(10.0);

		init();

		RCLCPP_INFO(nodeHandle->get_logger(), "Human Navi sample start!");

		auto subHumanNaviMsg    = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>         ("/human_navigation/message/to_robot", 100, std::bind(&HumanNavigationSample::messageCallback, this, std::placeholders::_1));
		auto subTaskInfoMsg     = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>    ("/human_navigation/message/task_info", 1, std::bind(&HumanNavigationSample::taskInfoMessageCallback, this, std::placeholders::_1));
		auto subAvatarStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>("/human_navigation/message/avatar_status", 1, std::bind(&HumanNavigationSample::avatarStatusMessageCallback, this, std::placeholders::_1));
		auto subObjectStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>("/human_navigation/message/object_status", 1, std::bind(&HumanNavigationSample::objectStatusMessageCallback, this, std::placeholders::_1));
		auto pubHumanNaviMsg    = nodeHandle->create_publisher   <human_navigation_msgs::msg::HumanNaviMsg>         ("/human_navigation/message/to_moderator", 10);
		auto pubGuidanceMsg     = nodeHandle->create_publisher   <human_navigation_msgs::msg::HumanNaviGuidanceMsg> ("/human_navigation/message/guidance_message", 10);

		timePrevSpeechStateConfirmed = nodeHandle->now();

		rclcpp::Time time(0, 0, nodeHandle->get_clock()->get_clock_type());

		while (rclcpp::ok())
		{
			switch(step)
			{
				case Initialize:
				{
					reset();

					RCLCPP_INFO(nodeHandle->get_logger(), "##### Initialized ######");

					step++;
					break;
				}
				case Ready:
				{
					if(isStarted)
					{
						step++;

						sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);

						RCLCPP_INFO(nodeHandle->get_logger(), "Task start");
					}
					break;
				}
				case WaitTaskInfo:
				{
					if(isTaskInfoReceived){ step++; }
					break;
				}
				case GuideForTakingObject:
				{
					if(isRequestReceived)
					{
						isRequestReceived = false;
					}

					std::string targetObjectName;
					if(taskInfo.target_object.name.find("empty_plastic_bottle") != std::string::npos)
					{
						targetObjectName = "an empty plastic bottle ";
					}
					else
					{
						targetObjectName = "a cup ";
					}

					std::string locationName;
					if(taskInfo.target_object.position.x > 0.0)
					{
						locationName = "on a table.";
					}
					else
					{
						locationName = "next to the kitchen sink.";
					}

					guideMsg = "Please take " + targetObjectName + locationName;

					if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
					{
						time = nodeHandle->now();
						step++;
					}
					break;
				}
				case GuideForPlacement:
				{
					if(isRequestReceived)
					{
						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 5;
					if(time.seconds() + WaitTime < nodeHandle->now().seconds())
					{
						std::string destinationName;
						if(taskInfo.destination.position.z < 1.0)
						{
							destinationName = "a trash can on the left.";
						}
						else
						{
							destinationName = "the second cabinet from the right.";
						}
						guideMsg = "Put it in " + destinationName;

						if(speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
						{
							time = nodeHandle->now();
							step++;
						}
					}

					break;
				}
				case WaitTaskFinished:
				{
					if(isFinished)
					{
						RCLCPP_INFO(nodeHandle->get_logger(), "Task finished");
						step++;
						break;
					}

					if(isRequestReceived)
					{
						bool isSpeaked;
						if((static_cast<int>(nodeHandle->now().seconds()) % 2) > 0)
						{
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg);
						}
						else
						{
							isSpeaked = speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, "You can find the wall cabinet above the kitchen sink.");
						}

						if(isSpeaked)
						{
							isRequestReceived = false;
						}
					}

					int WaitTime = 5;
					if(time.seconds() + WaitTime < nodeHandle->now().seconds())
					{
						if(!isSentGetAvatarStatus && !isSentGetObjectStatus)
						{
							sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
							sendMessage(pubHumanNaviMsg, MSG_GET_OBJECT_STATUS);
							isSentGetAvatarStatus = true;
							isSentGetObjectStatus = true;
							time = nodeHandle->now();
						}
					}

					break;
				}
				case TaskFinished:
				{
					// Wait MSG_GO_TO_NEXT_SESSION or MSG_MISSION_COMPLETE
					break;
				}
			}

			rclcpp::spin_some(nodeHandle);

			loopRate.sleep();
		}

		return 0;
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	HumanNavigationSample humanNaviSample;

	humanNaviSample.run(argc, argv);

	rclcpp::shutdown();
}

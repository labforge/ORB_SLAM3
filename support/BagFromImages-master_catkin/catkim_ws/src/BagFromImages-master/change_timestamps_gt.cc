#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv)
{
     // Initialize ROS
    ros::init(argc, argv, "change_timestamp");

    // Check if the input arguments are provided
    if (argc < 6)
    {
        ROS_ERROR("Usage: change_timestamp <input_bag_file> <output_bag_file> <fps> <ip groundtruth> <op groundtruth>");
        return 1;
    }

    // Get the input and output bag file names from the command line arguments
    std::string inputBagFile = argv[1];
    std::string outputBagFile = argv[2];
    std::string freq_arg = argv[3];
    std::string ip_groundtruth = argv[4];
    std::string op_groundtruth = argv[5];

    ros::start();

    ros::Time t = ros::Time::now();
    double freq = stof(freq_arg);
    const float T=1.0f/freq;
    ros::Duration d(T);
    int idx=0;

    // Vector to store all the values of t
    std::vector<ros::Time> tValues;

    try
    {
        // Open the input bag file for reading
        rosbag::Bag inputBag(inputBagFile, rosbag::bagmode::Read);

        // Create a view of the input bag file
        rosbag::View inputBagView(inputBag);

        // Create a bag file for writing the modified messages
        rosbag::Bag outputBag(outputBagFile, rosbag::bagmode::Write);

        // Iterate over the messages in the input bag file
        for (const rosbag::MessageInstance& msg : inputBagView)
        {
            // Update the timestamp in the message header
            std_msgs::Header::Ptr header = msg.instantiate<std_msgs::Header>();
            if (header != nullptr)
            {
                header->stamp = t;
            }

            // Write the modified message to the output bag file
            outputBag.write(msg.getTopic(), t, msg);

            if(msg.getTopic() ==  "/cam0/image_raw")
            {
                t+=d;
                idx++;
                std::cout << "idx:" << idx << std::endl;
                // Store the value of t in the vector
                tValues.push_back(t);
            }
        }

        // Close the bag files
        inputBag.close();
        outputBag.close();

        ROS_INFO("Timestamps modified successfully!");
    }
    catch (const rosbag::BagIOException& e)
    {
        ROS_ERROR("Error opening or reading the bag file: %s", e.what());
        return 1;
    }


    // Open the CSV file for reading
    std::ifstream inputFile(ip_groundtruth);
    
    if (!inputFile.is_open()) {
        std::cout << "Failed to open the CSV file." << std::endl;
        return 1;
    }
    
    // Read the file line by line
    std::string line;
    std::vector<std::vector<std::string>> data;
    
    while (std::getline(inputFile, line)) {
        std::istringstream iss(line);
        std::vector<std::string> row;
        std::string value;
        
        // Read each value in the line and store it in a vector
        while (std::getline(iss, value, ',')) {
            row.push_back(value);
        }
        
        data.push_back(row);
    }
    /*
    // Update the values in the first column
    for (auto& row : data) {
        if (!row.empty()) {
            row[0] = "New Value";  // Modify the value as desired
        }
    }
    */
    for (size_t i = 0; i < data.size(); ++i) {
        if (!data[i].empty() && i < tValues.size()) {
            data[i][0] = std::to_string(tValues[i].toSec());  // Convert ros::Time to string
        }
    }


    // Open the CSV file for writing
    std::ofstream outputFile("data_modified.csv");
    
    if (!outputFile.is_open()) {
        std::cout << "Failed to open the output file." << std::endl;
        return 1;
    }
    
    // Write the modified data back to the file
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            outputFile << row[i];
            
            // Add a comma after each value, except for the last one in the row
            if (i < row.size() - 1) {
                outputFile << ",";
            }
        }
        
        outputFile << std::endl;
    }
    
    // Close the files
    inputFile.close();
    outputFile.close();
    
    std::cout << "CSV file successfully modified." << std::endl;

    return 0;
}

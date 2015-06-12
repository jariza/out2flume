/*!
 *  \brief     Sends rosout_agg to Flume via Rest
 *  \author    José Jaime Ariza (jjaime@ikergune.com)
 *  \copyright See LICENSE file
 */

#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include "std_msgs/Header.h"
#include <sstream>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <curl/curl.h>

//!URL for the Flume REST service
std::string resturl;

/**
 * Curl write callback, we don't need the data so we won't store it
 */
size_t curlcb(void *ptr, size_t size, size_t nmemb, void *data) {
  return size * nmemb;
}

/**
 * Log callback
 * @param msg Message received
 */
void logCallback(const rosgraph_msgs::Log::ConstPtr& msg) {
    std::stringstream jsonmsg;
    std::string jsonstr, msgstr;
    CURL *curlh;
    CURLcode curlstatus;
    long http_code;

    //Process msg string
    msgstr = msg->msg;
    std::replace(msgstr.begin(), msgstr.end(), '"', '\'');
    std::replace(msgstr.begin(), msgstr.end(), '\n', ' ');
    msgstr.erase(std::remove(msgstr.begin(), msgstr.end(), '\r'), msgstr.end());

    //Create JSON message
    jsonmsg << "[{";
    jsonmsg << "\"headers\":{";
    jsonmsg << "\"timestamp\":" << msg->header.stamp.sec << std::fixed << std::setprecision(0) << (msg->header.stamp.nsec/1E6) << ",";
    jsonmsg << "\"exacttime\":" << msg->header.stamp.sec << "." << msg->header.stamp.nsec << ",";
    jsonmsg << "\"level\":" << (int)msg->level << ",";
    jsonmsg << "\"node\":\"" << msg->name << "\",";
    jsonmsg << "\"file\":\"" << msg->file << "\",";
    jsonmsg << "\"function\":\"" << msg->function << "\",";
    jsonmsg << "\"line\":" << msg->line;
    jsonmsg << "},";
    jsonmsg << "\"body\":\"" << msgstr << "\"";
    jsonmsg << "}]";

    //Send it to flume via REST
    curlh = curl_easy_init();
    if (!curlh) {
        fprintf(stderr, "curl_easy_init(): error, resturl: %s\n", resturl.c_str());
    }
    else {
        curl_easy_setopt(curlh, CURLOPT_URL, resturl.c_str());
        curl_easy_setopt(curlh, CURLOPT_WRITEFUNCTION, curlcb);
        jsonstr = jsonmsg.str(); //Temporal string and pointers problem: http://stackoverflow.com/questions/15155277/string-conversion-to-const-char-problems
        curl_easy_setopt(curlh, CURLOPT_POSTFIELDS, jsonstr.c_str());
        curl_easy_setopt(curlh, CURLOPT_POSTFIELDSIZE_LARGE, (curl_off_t)jsonstr.length());
        curlstatus = curl_easy_perform(curlh);
        if(curlstatus != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s, resturl: %s\n", curl_easy_strerror(curlstatus), resturl.c_str());
        }
        else {
            curl_easy_getinfo(curlh, CURLINFO_RESPONSE_CODE, &http_code);
            if (http_code != 200) {
                fprintf(stderr, "Return status not 200 (%li), resturl: %s\n", http_code, resturl.c_str());
            }
        }
    }
    curl_easy_cleanup(curlh);

}

int main(int argc, char **argv) {

    //Node initialization
    ros::init(argc, argv, "out2flume");

    //Get parameter
    if (ros::param::get("~resturl", resturl)) {
        //Typical subscriber initialization
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("rosout_agg", 1000, logCallback);

        //Let's go
        ros::spin();

        return 0;
    }
    else {
        fprintf(stderr, "resturl parameter missing\n");
        return 1;
    }
}

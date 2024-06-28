#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <limits.h>
// #include <algorithm>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000
#define EARTH_RADIUS 6371000.0 // Радиус Земли в метрах

const double DEVIATION_THRESHOLD = 0.5; // Variable to store deviation distance
const uint32_t CONSTANT_SPEED = 2;
const double WAYPOINT_REACHED_RADIUS = 1.0; // Radius in meters


extern uint32_t commandNum;
extern MissionCommand* commands;

// Declare changeWaypoint function
// extern int changeWaypoint(int32_t latitude, int32_t longitude, int32_t altitude);

// Function to calculate perpendicular distance from a point to a line segment in 3D space
double perpendicularDistance3D(double x, double y, double z, double x1, double y1, double z1, double x2, double y2, double z2) {
    double A = x - x1;
    double B = y - y1;
    double C = z - z1;
    double D = x2 - x1;
    double E = y2 - y1;
    double F = z2 - z1;
    double dot = A * D + B * E + C * F;
    double len_sq = D * D + E * E + F * F;
    double param = (len_sq != 0) ? (dot / len_sq) : -1;
    double xx, yy, zz;
    if (param < 0) {
        xx = x1;
        yy = y1;
        zz = z1;
    } else if (param > 1) {
        xx = x2;
        yy = y2;
        zz = z2;
    } else {
        xx = x1 + param * D;
        yy = y1 + param * E;
        zz = z1 + param * F;
    }
    double dx = x - xx;
    double dy = y - yy;
    double dz = z - zz;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Function to calculate the distance between two points in 3D space
double distance3D(double x1, double y1, double z1, double x2, double y2, double z2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}

int sendSignedMessage(const char* method, char* response, const char* errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);
    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);
    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    return 1;
}

// Function to find the next waypoint
uint32_t getNextWayPoint(uint32_t startIndex) {
    for (uint32_t i = startIndex; i < commandNum; i++) {
        if (commands[i].type == CommandType::WAYPOINT) {
            return i;
        }
    }
    return commandNum;
}

// Function to convert coordinates to real positions
void getPosition(int32_t latitude, int32_t longitude, int32_t altitude, double& posX, double& posY, double& posZ) {
    posX = (double)(latitude) / 1e7 * EARTH_RADIUS * M_PI / 180.0; // Convert to meters
    posY = (double)(longitude) / 1e7 * EARTH_RADIUS * M_PI / 180.0; // Convert to meters
    posZ = (double)(altitude) / 100.0; // Convert to meters from centimeters
}

// uint32_t getClosestSegment(double currentX, double currentY, double currentZ, double& minDistance) {
//     uint32_t closestSegment = 0;
//     minDistance = std::numeric_limits<double>::max();
//     for (uint32_t i = 1; i < commandNum; i++) {
//         if (commands[i].type != CommandType::WAYPOINT || commands[i - 1].type != CommandType::WAYPOINT) continue;
//         double segmentStartX, segmentStartY, segmentStartZ;
//         double segmentEndX, segmentEndY, segmentEndZ;
//         getPosition(commands[i - 1].content.waypoint.latitude, commands[i - 1].content.waypoint.longitude, commands[i - 1].content.waypoint.altitude, segmentStartX, segmentStartY, segmentStartZ);
//         getPosition(commands[i].content.waypoint.latitude, commands[i].content.waypoint.longitude, commands[i].content.waypoint.altitude, segmentEndX, segmentEndY, segmentEndZ);
//         double distance = perpendicularDistance3D(currentX, currentY, currentZ, segmentStartX, segmentStartY, segmentStartZ, segmentEndX, segmentEndY, segmentEndZ);
//         if (distance < minDistance) {
//             minDistance = distance;
//             closestSegment = i;
//         }
//     }
//     return closestSegment;
// }


int main(void) {
    // Ensure that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }

    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    // Enable buzzer to indicate that all modules have been initialized
    if (!enableBuzzer()) {
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);
    }

    // Copter needs to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    // Constantly ask server if mission for the drone is available. Parse it and ensure that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    // The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);

    while (true) {
        // Wait until autopilot wants to arm (and fails as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        // When autopilot asked for arm, we need to receive permission from ORVD
        char armResponse[1024] = {0};
        sendSignedMessage("/api/arm", armResponse, "arm", RETRY_DELAY_SEC);
        if (strstr(armResponse, "$Arm: 0#") != NULL) {
            // If arm was permitted, enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm()) {
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            }
            break;
        } else if (strstr(armResponse, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm()) {
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
            }
        } else {
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        }
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    }

    // If we get here, the drone is able to arm and start the mission
    // The flight needs to be controlled from now on
    // Also we need to check on ORVD whether the flight is still allowed or needs to be paused

    // Get initial home altitude
    int32_t initialAltitudeCm = commands[0].content.waypoint.altitude;
    double initialAltitudeM = (double)(initialAltitudeCm) / 100.0;

    // making 2 first points
    commands[0].content.waypoint.altitude = 0;
    commands[1].content.waypoint.latitude = commands[0].content.waypoint.latitude;
    commands[1].content.waypoint.longitud = commands[0].content.waypoint.longitude;

    // home point
    double prevWaypointX, prevWaypointY, prevWaypointZ;
    getPosition(commands[0].content.waypoint.latitude, commands[0].content.waypoint.longitude, commands[0].content.waypoint.altitude, prevWaypointX, prevWaypointY, prevWaypointZ);

    // takeoff point
    uint32_t currentWaypoint = 1;
    double waypointX = 0.0, waypointY = 0.0, waypointZ = 0.0;
    getPosition(commands[currentWaypoint].content.waypoint.latitude, commands[currentWaypoint].content.waypoint.longitude, commands[currentWaypoint].content.waypoint.altitude, waypointX, waypointY, waypointZ);
    
    double lastX = 0.0, lastY = 0.0, lastZ = 0.0;
    lastX = prevWaypointX;
    lastY = prevWaypointY;
    lastZ = prevWaypointZ;
    fprintf(stderr, "[%s] Info: Heading to waypoint %u at (%f, %f, %f)\n", ENTITY_NAME, currentWaypoint, waypointX, waypointY, waypointZ);

    double totalDistance = 0.0;
    time_t startTime;
    bool timerStarted = false;

    // Create an array to remember which waypoints need cargo drop
    uint32_t cargoDropWaypoints;
    bool isDroped = false;
    // 8 drop 10
    for (uint32_t i = 0; i < commandNum; i++) {
        if (commands[i].type == CommandType::SET_SERVO) {
            cargoDropWaypoints = getNextWayPoint(i + 1); // Store the waypoint after SET_SERVO command
        }
    }

    // Timer variables
    time_t lastPrintTime = time(NULL);

    while (currentWaypoint < commandNum) {
        int32_t latitude = 0, longitude = 0, altitudeCm = 0;
        if (!getCoords(latitude, longitude, altitudeCm)) {
            fprintf(stderr, "[%s] Warning: Lost connection with drone\n", ENTITY_NAME);
        } else {
            // Output received coordinates
            fprintf(stderr, "[%s] Info: Got coordinates: latitude: %d, longitude: %d, altitude: %d\n", ENTITY_NAME, latitude, longitude, altitudeCm);

            double currentX, currentY, currentZ;
            double altitudeM = (double)(altitudeCm) / 100.0 - initialAltitudeM; // Adjust altitude based on initial altitude
            getPosition(latitude, longitude, altitudeCm, currentX, currentY, currentZ);
            currentZ = altitudeM;

            if (!timerStarted) {
                startTime = time(NULL);
                timerStarted = true;
                fprintf(stderr, "[%s] Info: Timer started\n", ENTITY_NAME);
            }

            // Output current coordinates
            fprintf(stderr, "[%s] Current coordinates: latitude: %d, longitude: %d, altitude: %.2f meters\n", ENTITY_NAME, latitude, longitude, altitudeM);
            fprintf(stderr, "[%s] Following waypoint: %u\n", ENTITY_NAME, currentWaypoint);

            // Calculate distance from last position
            double segmentDistance = distance3D(lastX, lastY, lastZ, currentX, currentY, currentZ);
            totalDistance += segmentDistance;
            lastX = currentX;
            lastY = currentY;
            lastZ = currentZ;

            // Check if the current waypoint is reached
            double distanceToWaypoint = distance3D(currentX, currentY, currentZ, waypointX, waypointY, waypointZ);
            fprintf(stderr, "[%s] Debug: Distance to waypoint %u: %f meters\n", ENTITY_NAME, currentWaypoint, distanceToWaypoint);

            // Calculate the perpendicular distance to the current segment

            // uint32_t currentSegment = getClosestSegment(currentX, currentY, currentZ, minDistance);
            // fprintf(stderr, "[%s] Currently in segment: %u to %u\n", ENTITY_NAME, currentSegment - 1, currentSegment);
            
            double minDistance = perpendicularDistance3D(
                currentX,
                currentY, 
                currentZ, 
                prevWaypointX,
                prevWaypointY,
                prevWaypointZ,
                waypointX,
                waypointY,
                waypointZ
            );
            fprintf(stderr, "[%s] Perpendicular distance to segment: %.2f meters\n", ENTITY_NAME, minDistance);


            // Check if the perpendicular distance is greater than the deviation threshold
            if (minDistance > DEVIATION_THRESHOLD) {
                // Correct the path
                if (changeWaypoint(commands[currentWaypoint].content.waypoint.latitude,
                                   commands[currentWaypoint].content.waypoint.longitude,
                                   commands[currentWaypoint].content.waypoint.altitude) != 1) {
                    fprintf(stderr, "[%s] Warning: Failed to change waypoint\n", ENTITY_NAME);
                } else {
                    fprintf(stderr, "[%s] Info: Changed waypoint to (%d, %d, %d)\n", ENTITY_NAME,
                            commands[currentWaypoint].content.waypoint.latitude,
                            commands[currentWaypoint].content.waypoint.longitude,
                            commands[currentWaypoint].content.waypoint.altitude);
                }
            }

            if (distanceToWaypoint < WAYPOINT_REACHED_RADIUS) {
                fprintf(stderr, "[%s] Info: Waypoint %u reached. Waiting for 2 seconds before proceeding to next waypoint.\n", ENTITY_NAME, currentWaypoint);

                currentWaypoint = getNextWayPoint(currentWaypoint + 1);
                if (currentWaypoint < commandNum) {
                    prevWaypointX = waypointX;
                    prevWaypointY = waypointY;
                    prevWaypointZ = waypointZ;
                    getPosition(commands[currentWaypoint].content.waypoint.latitude,commands[currentWaypoint].content.waypoint.longitude, commands[currentWaypoint].content.waypoint.altitude, waypointX, waypointY, waypointZ);
                    fprintf(stderr, "[%s] Info: Heading to waypoint %u at (%f, %f, %f)\n", ENTITY_NAME, currentWaypoint, waypointX, waypointY, waypointZ);
                }
            }

            // Check if we need to release cargo at the next waypoint
            if (currentWaypoint == cargoDropWaypoints) {
                if (!setCargoLock(1)) {
                    fprintf(stderr, "[%s] Warning: Failed to enable cargo drop\n", ENTITY_NAME);
                } else {
                    fprintf(stderr, "[%s] Info: Cargo drop is enabled at waypoint %u\n", ENTITY_NAME);
                }
                isDroped = true;
            } else if (!isDroped) {
                // Ensure cargo drop is disabled when not at a drop point
                if (!setCargoLock(0)) {
                    fprintf(stderr, "[%s] Warning: Failed to ensure cargo drop is disabled\n", ENTITY_NAME);
                }
            }
        }

        // Ensure constant speed
        if (!changeSpeed(CONSTANT_SPEED)) {
            fprintf(stderr, "[%s] Warning: Failed to set constant speed\n", ENTITY_NAME);
        } else {
            fprintf(stderr, "[%s] Info: Speed set to %d m/s\n", ENTITY_NAME, CONSTANT_SPEED);
        }

        // Check if a second has passed and print the elapsed time
        time_t currentTime = time(NULL);
        if (difftime(currentTime, lastPrintTime) >= 1.0) {
            lastPrintTime = currentTime;
            double elapsedSeconds = difftime(currentTime, startTime);
            fprintf(stderr, "[%s] Elapsed time: %.0f seconds\n", ENTITY_NAME, elapsedSeconds);
        }

        // kill switch 
        if (orvd killSwitch) kill();

        // Delay for 500 milliseconds
        usleep(500000); // Delay for 500 milliseconds
    }

    kill();

    time_t endTime = time(NULL);
    double totalTime = difftime(endTime, startTime);
    fprintf(stderr, "[%s] Mission completed. Total distance: %.2f meters, Total time: %.2f seconds\n", ENTITY_NAME, totalDistance, totalTime);

    // Free the allocated memory for cargo drop waypoints
    free(cargoDropWaypoints);

    return EXIT_SUCCESS;
}

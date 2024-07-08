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
#include <string>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

const double EARTH_RADIUS = 6371000.0; // Earth radius
const double DEVIATION_THRESHOLD = 1; // Variable to store deviation distance
const double DEVIATION_ALTITUDE = 0.2; // Variable to store deviation distance
const uint32_t CONSTANT_SPEED = 3;
const double WAYPOINT_REACHED_RADIUS = 2; // Radius in meters
const uint32_t PING_DELAY_MLSEC = 500;
const double MAX_LANDING_TIME = 15;


extern uint32_t commandNum;
extern MissionCommand* commands;

struct Point {
    double x, y, z;
};

struct PointOrig {
    int32_t x, y, z;
};

// Function to convert coordinates to real positions
void convertPoint(PointOrig p0, Point& p1) {
    p1.x = ((double)p0.x) * M_PI * EARTH_RADIUS / 1e7 / 180.0; // Convert to meters
    p1.y = ((double)p0.y) * EARTH_RADIUS * M_PI / 1e7 / 180.0; // Convert to meters
    p1.z = ((double)p0.z) / 100.0; // Convert to meters from centimeters
}

void makePoints(Point* points, PointOrig* pointsOrig, int32_t homeSmZ) {
    uint32_t uk = 0;
    for (uint32_t i = 0; i < commandNum; i++) {
        if (commands[i].type == CommandType::SET_SERVO) continue;
        if (commands[i].type == CommandType::WAYPOINT) {
            pointsOrig[uk] = {
                commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude,
                commands[i].content.waypoint.altitude
            };
        }
        else if (commands[i].type == CommandType::HOME) {
            pointsOrig[uk] = {
                commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude,
                0
            };
        }
        else if (commands[i].type == CommandType::LAND) {
            pointsOrig[uk] = {
                commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude,
                commands[i].content.waypoint.altitude - homeSmZ
            };
        }
        else if (commands[i].type == CommandType::TAKEOFF) {
            pointsOrig[uk] = pointsOrig[uk - 1];
            pointsOrig[uk].z = commands[i].content.takeoff.altitude;
        }
        convertPoint(
            pointsOrig[uk],
            points[uk]
        );
        ++uk;
    }
}

// Function to calculate the distance between two points in 3D space
double distance3D(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

// Function to calculate the distance between two points in 2D space
double distance2D(Point p1, Point p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

// Function to calculate 2D closest point
template<typename P>
P getClosestPoint(P p, P f, P s) {
    long double A = p.x - f.x;
    long double B = p.y - f.y;
    long double D = s.x - f.x;
    long double E = s.y - f.y;
    long double F = s.z - f.z;
    long double dot = A * D + B * E;
    long double len_sq = D * D + E * E;
    long double param = (len_sq != 0) ? (dot / len_sq) : 2;
    P closest;
    if (param < 0) {
        closest = f;
    } else if (param > 1) {
        closest = s;
    } else {
        closest = {f.x + param * D, f.y + param * E, f.z + param * F};
    }
    return closest;
}

bool isKilled = false;

void kill() {
    if (isKilled) return;
    fprintf(stderr, "[%s] Killing dron\n", ENTITY_NAME);
    enableBuzzer();
    setKillSwitch(0);
    isKilled = true;
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

bool isKillSwitch() {
    const int32_t MAX_PINGS = 1;
    char killResponse[1024] = {0};
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", "/api/kill_switch", BOARD_ID);
    int32_t numPings = MAX_PINGS;
    while (!signMessage(message, signature) && --numPings) {
        fprintf(stderr, "[%s] Warning: Failed to sign killSwitch message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        if (--numPings) return false;
        sleep(RETRY_DELAY_SEC);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);
    numPings = MAX_PINGS;
    while (!sendRequest(request, killResponse)) {
        fprintf(stderr, "[%s] Warning: Failed to send killSwitch request through Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        if (--numPings) return false;
        sleep(RETRY_DELAY_SEC);
    }
    uint8_t authenticity = 0;
    numPings = MAX_PINGS;
    while (!checkSignature(killResponse, authenticity) || !authenticity) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of killSwitch response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        if (--numPings) return false;
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] KillSwitch response: %s \n", ENTITY_NAME, killResponse);
    return (strstr(killResponse, "$KillSwitch: 0#") != NULL);
}


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

    // Get initial home altitude
    int32_t homeSmZ = commands[0].content.waypoint.altitude;
    double startZ = (double)(homeSmZ) / 100.0;

    Point* points = (Point*)malloc((commandNum - 1) * sizeof(Point));
    PointOrig* pointsOrig = (PointOrig*)malloc((commandNum - 1) * sizeof(PointOrig));
    makePoints(points, pointsOrig, homeSmZ);
    fprintf(stderr, "[%s] Parsed path in metres\n", ENTITY_NAME);
    for (uint32_t i = 0; i < commandNum - 1; ++i) {
        fprintf(stderr, "[%s] Print waypoint %u at (%.2f, %.2f, %.2f)\n", ENTITY_NAME, i, points[i].x, points[i].y, points[i].z);
    }

    // takeoff point
    uint32_t curWaypoint = 1;
   
    Point lastMoment = points[0];
    fprintf(stderr, "[%s] Info: Heading to waypoint %u at (%.2f, %.2f, %.2f)\n", ENTITY_NAME, curWaypoint, points[curWaypoint].x, points[curWaypoint].y, points[curWaypoint].z);

    double totalDistance = 0.0;
    // Timer variables
    time_t lastPrintTime = time(NULL);
    time_t startTime;
    bool timerStarted = false;
    time_t landingTime;
    bool landingStarted = false;

    // finding drop waypoint
    uint32_t cargoDropWaypoint;
    bool isDroped = false;
    for (uint32_t i = 0; i < commandNum; i++) {
        if (commands[i].type == CommandType::SET_SERVO) {
            cargoDropWaypoint = i;
        }
    }

    while (curWaypoint < commandNum - 1) {
        struct timespec tw = {0, (uint32_t)1e6 * PING_DELAY_MLSEC};
        struct timespec tr;
        nanosleep(&tw, &tr);

        // Ensure constant speed
        if (!changeSpeed(CONSTANT_SPEED)) {
            fprintf(stderr, "[%s] Warning: Failed to set constant speed\n", ENTITY_NAME);
        } else {
            fprintf(stderr, "[%s] Info: Speed set to %d m/s\n", ENTITY_NAME, CONSTANT_SPEED);
        }

        // Check if a second has passed and print the elapsed time
        time_t curTime = time(NULL);
        if (difftime(curTime, lastPrintTime) >= 1.0) {
            lastPrintTime = curTime;
            double elapsedSeconds = difftime(curTime, startTime);
            fprintf(stderr, "[%s] Elapsed time: %.0f seconds\n", ENTITY_NAME, elapsedSeconds);
        }

        // checking killSitch from ORVD
        if (isKillSwitch()) {
            kill();
            break;
        }

        // getting coords
        PointOrig curOrig;
        if (!getCoords(curOrig.x, curOrig.y, curOrig.z)) {
            fprintf(stderr, "[%s] Warning: Lost connection with drone\n", ENTITY_NAME);
            continue;
        }
        // Output received coordinates
        fprintf(stderr, "[%s] Info: Got coordinates: latitude: %d, longitude: %d, altitude: %d\n", ENTITY_NAME, curOrig.x, curOrig.y, curOrig.z);

        // converting coords
        Point cur;
        convertPoint(curOrig, cur);
        cur.z = (double)(curOrig.z) / 100.0 - startZ;

        // Output cur coordinates
        fprintf(stderr, "[%s] Cur coordinates: latitude: %.2f, longitude: %.2f, altitude: %.2f meters\n", ENTITY_NAME, cur.x, cur.y, cur.z);
        fprintf(stderr, "[%s] Following waypoint: %u at (%.2f, %.2f, %.2f)\n", ENTITY_NAME, curWaypoint, points[curWaypoint].x, points[curWaypoint].y, points[curWaypoint].z);


        // starting global timer
        if (!timerStarted) {
            startTime = time(NULL);
            timerStarted = true;
            fprintf(stderr, "[%s] Info: Timer started\n", ENTITY_NAME);
        }

        // Calculate distance from last position
        double distFromLastMoment = distance3D(lastMoment, cur);
        totalDistance += distFromLastMoment;
        lastMoment = cur;

        // distance left
        double distanceToWaypoint = distance3D(cur, points[curWaypoint]);
        fprintf(stderr, "[%s] Debug: Distance to waypoint %u: %.2f meters\n", ENTITY_NAME, curWaypoint, distanceToWaypoint);

        // getting closest point
        Point pClosest = getClosestPoint(cur, points[curWaypoint - 1], points[curWaypoint]);
        PointOrig pClosestOrig = getClosestPoint(curOrig, pointsOrig[curWaypoint - 1], pointsOrig[curWaypoint]);
        
        // distance to segment
        double minDistance = distance2D(cur, pClosest);
        double altitudeDeviation = abs(pClosest.z - cur.z);
        fprintf(stderr, "[%s] Perpendicular distance to segment: %.2f meters\n", ENTITY_NAME, minDistance);
        fprintf(stderr, "[%s] Altitude deviation threshold: %.2f meters\n", ENTITY_NAME, altitudeDeviation);

        // Correcting x and y coordinates
        if (minDistance > DEVIATION_THRESHOLD) {
            // Correct the path
            if (changeWaypoint(pClosestOrig.x, pClosestOrig.y, pClosestOrig.z)) {
                fprintf(stderr, "[%s] Info: Changed waypoint to (%d, %d, %d)\n", ENTITY_NAME,
                    pClosestOrig.x, pClosestOrig.y, pClosestOrig.z);
                fprintf(stderr, "[%s] In metres (%.2f, %.2f, %.2f)\n", ENTITY_NAME,
                    pClosest.x, pClosest.y, pClosest.z);
            } else {
                fprintf(stderr, "[%s] Warning: Failed to change waypoint\n", ENTITY_NAME);
            }
        }

        bool isLanding = (
            pointsOrig[curWaypoint].x == pointsOrig[curWaypoint - 1].x && 
            pointsOrig[curWaypoint].y == pointsOrig[curWaypoint - 1].y &&
            pointsOrig[curWaypoint].z < pointsOrig[curWaypoint - 1].z
        );

        // Correcting altitude
        if (altitudeDeviation > DEVIATION_ALTITUDE && !isLanding) {
            if (changeAltitude(pClosestOrig.z)) {
                fprintf(stderr, "[%s] Info: Changed altitude to %d\n", ENTITY_NAME, pClosestOrig.z);
            } else {
                fprintf(stderr, "[%s] Warning: Failed to change altitude\n", ENTITY_NAME);
            }
        }

        if (isLanding && !landingStarted) {
            landingStarted = true;
            landingTime = time(NULL);
        } else if (isLanding) {
            if (difftime(time(NULL), landingTime) >= MAX_LANDING_TIME) {
                fprintf(stderr, "[%s] Too long landing\n", ENTITY_NAME);
                kill();
                break;
            }
        } else {
            landingStarted = false;
        }

        // Check if the cur waypoint is reached
        if (distanceToWaypoint <= WAYPOINT_REACHED_RADIUS) {
            fprintf(stderr, "[%s] Info: Waypoint %u reached!!!\n", ENTITY_NAME, curWaypoint);
            ++curWaypoint;
        }

        // Check if we need to release cargo at the next waypoint
        if (!isDroped && (curWaypoint == cargoDropWaypoint || (curWaypoint == cargoDropWaypoint - 1 && distanceToWaypoint <= 2 * WAYPOINT_REACHED_RADIUS))) {
            if (setCargoLock(1)) {
                fprintf(stderr, "[%s] Info: Cargo drop is enabled at waypoint %u\n", ENTITY_NAME, curWaypoint);
            } else {
                fprintf(stderr, "[%s] Warning: Failed to enable cargo drop\n", ENTITY_NAME);
            }
            isDroped = true;
        } else if (!isDroped) {
            // Ensure cargo drop is disabled when not at a drop point
            if (!setCargoLock(0)) {
                fprintf(stderr, "[%s] Warning: Failed to ensure cargo drop is disabled\n", ENTITY_NAME);
            }
        }
    }
    sleep(5);
    kill();

    time_t endTime = time(NULL);
    double totalTime = difftime(endTime, startTime);
    fprintf(stderr, "[%s] Mission completed. Total distance: %.2f meters, Total time: %.2f seconds\n", ENTITY_NAME, totalDistance, totalTime);

    free(points);
    free(pointsOrig);

    return EXIT_SUCCESS;
}
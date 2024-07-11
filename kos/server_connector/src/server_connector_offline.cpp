#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int initServerConnector() {
    return 1;
}

int sendRequest(char* query, char* response) {
    if (strstr(query, "/api/kill_switch?") != NULL)
        strcpy(response, "$KillSwitch: 1#");
    else if (strstr(query, "/api/auth?") != NULL)
        strcpy(response, "$Success#");
    else if (strstr(query, "/api/fmission_kos?") != NULL)
        strcpy(response, "$FlightMission H46.6143809_142.8119646_98.20&T2.0&W0.0_46.61411540_142.81191400_2.5&W0.0_46.61412780_142.81181630_2.5&W0.0_46.61432130_142.81174470_1.5&W0.0_46.61415910_142.81157170_2.5&W0.0_46.61444190_142.81158040_2.5&W0.0_46.61444190_142.81158040_1.0&S5.0_1200.0&W0.0_46.61444190_142.81158040_2.5&W0.0_46.61438090_142.81196460_2.5&L46.61438090_142.81196460_98.20#");
    else if ((strstr(query, "/api/arm?") != NULL) || (strstr(query, "/api/fly_accept?") != NULL))
        strcpy(response, "$Arm: 0#");
    else
        strcpy(response, "$#");

    return 1;
}
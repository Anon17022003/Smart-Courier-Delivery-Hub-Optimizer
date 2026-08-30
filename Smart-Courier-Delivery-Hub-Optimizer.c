/*
 * ==========================================================================
 *  SMART COURIER & DELIVERY HUB OPTIMIZER
 *  Algorithms Lab Project
 *
 *  Core Algorithms:
 *    1. Dijkstra's Algorithm  -> real shortest road-distance between any
 *       two locations, even when they are not directly connected (routes
 *       through intermediate stops).
 *    2. Greedy Nearest-Neighbor Heuristic -> builds a delivery route by
 *       always moving to the closest unvisited delivery point next
 *       (classic approximate solution to the Traveling Salesperson
 *       Problem, used because exact TSP is too slow to compute exactly
 *       once the number of delivery points grows).
 *
 *  Comparison mode: greedy optimized route vs several random routes,
 *  showing distance & fuel-cost savings.
 *
 *  NEW IN THIS VERSION:
 *    - User account system (name, phone number, age) with register/login.
 *    - Customer "Estimate & Receive Delivery" flow: computes the SHORTEST
 *      distance from the Hub directly to the customer's receiving point
 *      (Dijkstra), then shows the total cost to pay and the estimated
 *      wait time, and finally requires a 4-digit OTP to confirm the
 *      package was actually received.
 *
 *  Edge cases handled:
 *    - fewer than 1 delivery point (nothing to optimize)
 *    - duplicate location names
 *    - self-loop roads (location connected to itself)
 *    - non-positive road distances
 *    - locations disconnected from the Hub (unreachable) -- Dijkstra
 *      reports them as unreachable, and the route optimizer skips them
 *      and reports which ones were skipped, instead of crashing or
 *      silently producing a wrong route
 *    - invalid location indices
 *    - maximum location capacity reached
 *    - duplicate phone numbers on registration
 *    - login required before receiving a delivery
 *    - wrong/incorrect OTP entries (limited retries)
 * ==========================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <float.h>

#define MAX_LOC     50
#define NAME_LEN    30
#define PHONE_LEN   16
#define INF         1e9

#define MAX_USERS       100
#define AVG_SPEED_KMH   20.0   /* realistic avg. courier speed in Dhaka traffic */
#define OTP_MAX_TRIES   3

/* -------------------------------------------------------------------
 *  Graph representation: weighted, undirected adjacency matrix.
 *  Location index 0 is always the Hub.
 * ------------------------------------------------------------------- */

typedef struct {
    int    numLoc;
    char   names[MAX_LOC][NAME_LEN];
    double adj[MAX_LOC][MAX_LOC];
    double fuelRate;   /* Tk per km */
} Graph;

typedef struct {
    int    route[MAX_LOC + 2];
    int    routeLen;
    double totalDistance;
    int    skipped[MAX_LOC];
    int    skippedCount;
    int    couldNotReturn;   /* 1 if it could not get back to hub */
} RouteResult;

/* -------------------------------------------------------------------
 *  User accounts
 * ------------------------------------------------------------------- */

typedef struct {
    char name[NAME_LEN];
    char phone[PHONE_LEN];
    int  age;
    int  favLocation;   /* index of favorite receiving point, -1 = none set */
} User;

static User users[MAX_USERS];
static int  userCount = 0;
static int  loggedInUserIdx = -1;   /* -1 = nobody logged in */

/* -------------------------------------------------------------------
 *  Setup
 * ------------------------------------------------------------------- */

static void initGraph(Graph *g) {
    g->numLoc = 1;
    strcpy(g->names[0], "DIU_Hub");
    g->fuelRate = 15.0;   /* Tk per km, editable via menu option 7 */
    for (int i = 0; i < MAX_LOC; i++)
        for (int j = 0; j < MAX_LOC; j++)
            g->adj[i][j] = (i == j) ? 0.0 : INF;
}

static int addLocation(Graph *g, const char *name) {
    if (g->numLoc >= MAX_LOC) return -1;              /* capacity full */
    for (int i = 0; i < g->numLoc; i++)
        if (strcmp(g->names[i], name) == 0) return -2; /* duplicate */
    strncpy(g->names[g->numLoc], name, NAME_LEN - 1);
    g->names[g->numLoc][NAME_LEN - 1] = '\0';
    g->numLoc++;
    return g->numLoc - 1;
}

/* returns: -1 invalid index, -2 self loop, -3 bad distance,
 *           0 new road added, 1 existing road updated */
static int addRoad(Graph *g, int u, int v, double dist) {
    if (u < 0 || u >= g->numLoc || v < 0 || v >= g->numLoc) return -1;
    if (u == v) return -2;
    if (dist <= 0) return -3;
    int updated = (g->adj[u][v] < INF);
    g->adj[u][v] = dist;
    g->adj[v][u] = dist;
    return updated ? 1 : 0;
}

/* -------------------------------------------------------------------
 *  DEFAULT NETWORK: Hub = Daffodil International University (DIU),
 *  Daffodil Smart City, Birulia, Ashulia, Savar, Dhaka-1216.
 *
 *  Road distances below are approximate real-road driving distances
 *  (not straight-line "as the crow flies" distances) along the actual
 *  routes a courier would take in Dhaka, e.g. the Ashulia-Tongi
 *  diversion road, the Dhaka-Aricha highway, the Airport Road, and the
 *  Mirpur-Gulshan-Dhanmondi link roads. They are estimates based on
 *  publicly known Dhaka road geography and typical Google Maps driving
 *  distances between these areas, not a live Google Maps API call
 *  (this program has no internet access). Use menu options 1 and 2 to
 *  add/correct any location or road distance with exact figures if you
 *  pull them from Google Maps yourself.
 *
 *  Not every pair of points has a direct road — some (like Tongi or
 *  Dhamrai to the far side of the city) are only reachable through one
 *  or more intermediate stops, which is exactly what lets Dijkstra's
 *  algorithm show its value: it finds the true shortest path even when
 *  it has to hop through other locations first.
 * ------------------------------------------------------------------- */

static void seedDefaultNetwork(Graph *g) {
    int hub      = 0;                                  /* DIU_Hub */
    int ashulia  = addLocation(g, "Ashulia");
    int tongi    = addLocation(g, "Tongi");
    int uttara   = addLocation(g, "Uttara");
    int mirpur   = addLocation(g, "Mirpur");
    int dhamrai  = addLocation(g, "Dhamrai");
    int banani   = addLocation(g, "Banani");
    int gulshan  = addLocation(g, "Gulshan");
    int dhanmondi= addLocation(g, "Dhanmondi");

    /* Hub's immediate neighbors (roads that leave the Ashulia area) */
    addRoad(g, hub,     ashulia,  4.0);   /* DIU campus -> Ashulia Bazar/Chowrasta   */
    addRoad(g, hub,     dhamrai,  15.0);  /* via Dhaka-Aricha Highway                 */
    addRoad(g, hub,     tongi,    16.0);  /* via Ashulia-Tongi diversion road         */

    /* Onward roads deeper into the city */
    addRoad(g, ashulia, uttara,   10.0);  /* via Ashulia-Diabari road                 */
    addRoad(g, tongi,   uttara,   10.0);  /* via Dhaka-Mymensingh Hwy / Airport Road  */
    addRoad(g, uttara,  mirpur,    9.0);  /* via Mirpur DOHS / Airport Road link      */
    addRoad(g, uttara,  banani,    8.0);  /* via Airport Road                         */
    addRoad(g, mirpur,  dhanmondi, 8.0);  /* via Mirpur Road / Kazi Nazrul Islam Ave  */
    addRoad(g, mirpur,  banani,   10.0);  /* via Mirpur-Kuril link road               */
    addRoad(g, banani,  gulshan,   3.0);  /* adjacent neighborhoods                   */
    addRoad(g, gulshan, dhanmondi, 9.0);  /* via Hatirjheel / Gulshan link road       */
    addRoad(g, dhamrai, mirpur,   20.0);  /* via Dhaka-Aricha Hwy -> Gabtoli -> Mirpur*/
}

static void dijkstra(Graph *g, int src, double dist[], int parent[]) {
    int visited[MAX_LOC] = {0};
    for (int i = 0; i < g->numLoc; i++) { dist[i] = INF; parent[i] = -1; }
    dist[src] = 0;

    for (int count = 0; count < g->numLoc; count++) {
        int u = -1;
        double best = INF;
        for (int i = 0; i < g->numLoc; i++)
            if (!visited[i] && dist[i] < best) { best = dist[i]; u = i; }

        if (u == -1) break;   /* everything left is unreachable */
        visited[u] = 1;

        for (int v = 0; v < g->numLoc; v++) {
            if (!visited[v] && g->adj[u][v] < INF) {
                double nd = dist[u] + g->adj[u][v];
                if (nd < dist[v]) { dist[v] = nd; parent[v] = u; }
            }
        }
    }
}

static void printPath(Graph *g, int parent[], int dest) {
    if (dest == -1) return;
    if (parent[dest] != -1) {
        printPath(g, parent, parent[dest]);
        printf(" -> ");
    }
    printf("%s", g->names[dest]);
}

static void showShortestPathsFromHub(Graph *g) {
    if (g->numLoc < 2) { printf("No delivery points added yet.\n"); return; }

    double dist[MAX_LOC];
    int parent[MAX_LOC];
    dijkstra(g, 0, dist, parent);

    printf("\n--- Shortest Paths from Hub (Dijkstra's Algorithm) ---\n");
    int anyUnreachable = 0;
    for (int i = 1; i < g->numLoc; i++) {
        if (dist[i] >= INF) {
            printf("%-15s : UNREACHABLE from Hub\n", g->names[i]);
            anyUnreachable = 1;
        } else {
            printf("%-15s : %7.2f km   [Path: ", g->names[i], dist[i]);
            printPath(g, parent, i);
            printf("]\n");
        }
    }
    if (anyUnreachable)
        printf("\nNote: unreachable locations have no road path back to the Hub yet.\n");
}

/* All-pairs shortest distances, built by running Dijkstra from every node.
 * This is what lets the route optimizer treat the road network as if it
 * had a "real" distance between any two delivery points, even when no
 * direct road connects them. */
static void computeAllPairs(Graph *g, double distMat[MAX_LOC][MAX_LOC]) {
    for (int i = 0; i < g->numLoc; i++) {
        double dist[MAX_LOC];
        int parent[MAX_LOC];
        dijkstra(g, i, dist, parent);
        for (int j = 0; j < g->numLoc; j++) distMat[i][j] = dist[j];
    }
}

/* -------------------------------------------------------------------
 *  GREEDY NEAREST-NEIGHBOR ROUTE OPTIMIZATION
 * ------------------------------------------------------------------- */

static RouteResult greedyNearestNeighbor(Graph *g, double distMat[MAX_LOC][MAX_LOC]) {
    RouteResult r;
    r.routeLen = 0;
    r.totalDistance = 0;
    r.skippedCount = 0;
    r.couldNotReturn = 0;

    if (g->numLoc < 2) return r;   /* nothing to deliver */

    int visited[MAX_LOC] = {0};
    visited[0] = 1;
    r.route[r.routeLen++] = 0;
    int current = 0;
    int remaining = g->numLoc - 1;

    while (remaining > 0) {
        int next = -1;
        double best = INF;
        for (int v = 1; v < g->numLoc; v++) {
            if (!visited[v] && distMat[current][v] < best) {
                best = distMat[current][v];
                next = v;
            }
        }

        if (next == -1) {
            /* Every remaining delivery point is unreachable from here. */
            for (int v = 1; v < g->numLoc; v++)
                if (!visited[v]) {
                    r.skipped[r.skippedCount++] = v;
                    visited[v] = 1;
                }
            break;
        }

        visited[next] = 1;
        r.totalDistance += best;
        r.route[r.routeLen++] = next;
        current = next;
        remaining--;
    }

    if (current != 0) {
        if (distMat[current][0] < INF) {
            r.totalDistance += distMat[current][0];
            r.route[r.routeLen++] = 0;
        } else {
            r.couldNotReturn = 1;
        }
    }

    return r;
}

static void printRoute(Graph *g, RouteResult *r) {
    if (r->routeLen <= 1) {
        printf("No reachable delivery points from Hub.\n");
        return;
    }

    printf("Route: ");
    for (int i = 0; i < r->routeLen; i++) {
        printf("%s", g->names[r->route[i]]);
        if (i < r->routeLen - 1) printf(" -> ");
    }
    if (r->couldNotReturn) printf("  [WARNING: could not return to Hub]");
    printf("\n");

    printf("Total Distance    : %.2f km\n", r->totalDistance);
    printf("Estimated Fuel Cost: Tk. %.2f\n", r->totalDistance * g->fuelRate);

    if (r->skippedCount > 0) {
        printf("Skipped (unreachable) locations: ");
        for (int i = 0; i < r->skippedCount; i++) {
            printf("%s", g->names[r->skipped[i]]);
            if (i < r->skippedCount - 1) printf(", ");
        }
        printf("\n");
    }
}

/* -------------------------------------------------------------------
 *  RANDOM ROUTE (for comparison mode)
 * ------------------------------------------------------------------- */

static double randomRouteDistance(Graph *g, double distMat[MAX_LOC][MAX_LOC],
                                   int reachable[], int reachCount,
                                   int outRoute[], int *outLen) {
    int perm[MAX_LOC];
    memcpy(perm, reachable, reachCount * sizeof(int));

    /* Fisher-Yates shuffle */
    for (int i = reachCount - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int tmp = perm[i]; perm[i] = perm[j]; perm[j] = tmp;
    }

    double total = 0;
    int current = 0;
    outRoute[0] = 0;
    *outLen = 1;

    for (int i = 0; i < reachCount; i++) {
        total += distMat[current][perm[i]];
        current = perm[i];
        outRoute[(*outLen)++] = current;
    }
    total += distMat[current][0];
    outRoute[(*outLen)++] = 0;

    (void) g;
    return total;
}

static void compareRoutes(Graph *g) {
    if (g->numLoc < 2) { printf("Add delivery points first.\n"); return; }

    double distMat[MAX_LOC][MAX_LOC];
    computeAllPairs(g, distMat);

    RouteResult greedy = greedyNearestNeighbor(g, distMat);
    if (greedy.routeLen <= 1) {
        printf("No reachable delivery points from Hub - cannot compare.\n");
        return;
    }

    int reachable[MAX_LOC], reachCount = 0;
    for (int i = 0; i < greedy.routeLen; i++) {
        int node = greedy.route[i];
        if (node > 0) reachable[reachCount++] = node;
    }
    if (reachCount == 0) {
        printf("No reachable delivery points - cannot compare.\n");
        return;
    }

    printf("\n=========== ROUTE COMPARISON ===========\n");
    printf("\n[GREEDY - Nearest Neighbor Optimized Route]\n");
    printRoute(g, &greedy);

    const int TRIALS = 5;
    double totalRandomDist = 0, bestRandom = DBL_MAX, worstRandom = 0;

    printf("\n[RANDOM ROUTES] (%d trials, for comparison)\n", TRIALS);
    for (int t = 0; t < TRIALS; t++) {
        int outRoute[MAX_LOC + 2], outLen;
        double d = randomRouteDistance(g, distMat, reachable, reachCount, outRoute, &outLen);

        printf("Trial %d: ", t + 1);
        for (int i = 0; i < outLen; i++) {
            printf("%s", g->names[outRoute[i]]);
            if (i < outLen - 1) printf(" -> ");
        }
        printf("   | Distance: %.2f km | Fuel: Tk. %.2f\n", d, d * g->fuelRate);

        totalRandomDist += d;
        if (d < bestRandom) bestRandom = d;
        if (d > worstRandom) worstRandom = d;
    }
    double avgRandom = totalRandomDist / TRIALS;

    printf("\n--- Summary ---\n");
    printf("Greedy Optimized Distance : %7.2f km  (Fuel: Tk. %.2f)\n",
           greedy.totalDistance, greedy.totalDistance * g->fuelRate);
    printf("Random Average Distance   : %7.2f km  (Fuel: Tk. %.2f)\n",
           avgRandom, avgRandom * g->fuelRate);
    printf("Random Best Distance      : %7.2f km\n", bestRandom);
    printf("Random Worst Distance     : %7.2f km\n", worstRandom);

    if (avgRandom > 0) {
        double savings = (avgRandom - greedy.totalDistance) / avgRandom * 100.0;
        printf("\n>>> Greedy route saves %.2f%% distance (and fuel cost) vs the average random route.\n",
               savings);
    }
    printf("=========================================\n");
}

/* -------------------------------------------------------------------
 *  Misc display
 * ------------------------------------------------------------------- */

static void viewNetwork(Graph *g) {
    printf("\n--- Locations ---\n");
    for (int i = 0; i < g->numLoc; i++)
        printf("[%d] %s%s\n", i, g->names[i], i == 0 ? "  (HUB)" : "");

    printf("\n--- Roads ---\n");
    int any = 0;
    for (int i = 0; i < g->numLoc; i++)
        for (int j = i + 1; j < g->numLoc; j++)
            if (g->adj[i][j] < INF) {
                printf("%s <-> %s : %.2f km\n", g->names[i], g->names[j], g->adj[i][j]);
                any = 1;
            }
    if (!any) printf("(No roads added yet.)\n");
}

/* -------------------------------------------------------------------
 *  Input helpers (defensive against bad input types)
 * ------------------------------------------------------------------- */

static void flushInput(void) {
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}
}

static int readInt(const char *prompt) {
    int v;
    printf("%s", prompt);
    while (scanf("%d", &v) != 1) {
        printf("Invalid input. Enter an integer: ");
        flushInput();
    }
    flushInput();
    return v;
}

static double readDouble(const char *prompt) {
    double v;
    printf("%s", prompt);
    while (scanf("%lf", &v) != 1) {
        printf("Invalid input. Enter a number: ");
        flushInput();
    }
    flushInput();
    return v;
}

static void readName(const char *prompt, char *buf, int maxLen) {
    printf("%s", prompt);
    if (!fgets(buf, maxLen, stdin)) { buf[0] = '\0'; return; }
    size_t len = strlen(buf);
    if (len > 0 && buf[len - 1] == '\n') buf[len - 1] = '\0';
}

/* -------------------------------------------------------------------
 *  USER ACCOUNTS: register / login
 * ------------------------------------------------------------------- */

static int findUserByPhone(const char *phone) {
    for (int i = 0; i < userCount; i++)
        if (strcmp(users[i].phone, phone) == 0) return i;
    return -1;
}

static void registerUser(void) {
    if (userCount >= MAX_USERS) {
        printf("Error: maximum number of accounts (%d) reached.\n", MAX_USERS);
        return;
    }

    User u;
    readName("Enter your full name: ", u.name, NAME_LEN);
    if (strlen(u.name) == 0) { printf("Error: name cannot be empty.\n"); return; }

    readName("Enter your phone number: ", u.phone, PHONE_LEN);
    if (strlen(u.phone) == 0) { printf("Error: phone number cannot be empty.\n"); return; }
    if (findUserByPhone(u.phone) != -1) {
        printf("Error: an account with phone number '%s' already exists.\n", u.phone);
        return;
    }

    u.age = readInt("Enter your age: ");
    if (u.age <= 0 || u.age > 120) { printf("Error: please enter a realistic age.\n"); return; }

    u.favLocation = -1;   /* no favorite receiving point yet */

    users[userCount] = u;
    loggedInUserIdx = userCount;
    userCount++;

    printf("Account created successfully! Welcome, %s. You are now logged in.\n", u.name);
}

static void loginUser(void) {
    if (userCount == 0) {
        printf("No accounts exist yet. Please register first.\n");
        return;
    }
    char phone[PHONE_LEN];
    readName("Enter your phone number to log in: ", phone, PHONE_LEN);
    int idx = findUserByPhone(phone);
    if (idx == -1) {
        printf("No account found with that phone number. Please register first.\n");
        return;
    }
    loggedInUserIdx = idx;
    printf("Welcome back, %s!\n", users[idx].name);
}

static void logoutUser(void) {
    if (loggedInUserIdx == -1) { printf("No user is currently logged in.\n"); return; }
    printf("Goodbye, %s. You have been logged out.\n", users[loggedInUserIdx].name);
    loggedInUserIdx = -1;
}

static void showAccountStatus(Graph *g) {
    if (loggedInUserIdx == -1) {
        printf("Status: not logged in.\n");
    } else {
        User *u = &users[loggedInUserIdx];
        printf("Logged in as: %s | Phone: %s | Age: %d\n", u->name, u->phone, u->age);
        if (u->favLocation == -1)
            printf("Favorite receiving point: not set (menu option 12)\n");
        else
            printf("Favorite receiving point: %s\n", g->names[u->favLocation]);
    }
}

/* -------------------------------------------------------------------
 *  FAVORITE RECEIVING POINT
 *  Lets a logged-in user save a preferred delivery location (e.g.
 *  "Uttara") so they don't have to pick it from the list every time
 *  they want to receive a package.
 * ------------------------------------------------------------------- */

static void setFavoriteLocation(Graph *g) {
    if (loggedInUserIdx == -1) {
        printf("Please log in or register an account first (menu option 9 or 10).\n");
        return;
    }
    if (g->numLoc < 2) {
        printf("No delivery points have been added yet.\n");
        return;
    }

    printf("\nAvailable receiving points: ");
    for (int i = 1; i < g->numLoc; i++) printf("[%d]%s ", i, g->names[i]);
    printf("\n");

    int idx = readInt("Enter the index of your favorite receiving point: ");
    if (idx <= 0 || idx >= g->numLoc) {
        printf("Error: invalid location index.\n");
        return;
    }

    users[loggedInUserIdx].favLocation = idx;
    printf("'%s' has been saved as your favorite receiving point.\n", g->names[idx]);
}

/* -------------------------------------------------------------------
 *  OTP-BASED RECEIVING VERIFICATION
 * ------------------------------------------------------------------- */

/* Generates a random 4-digit OTP in the range 1000-9999. */
static int generateOTP(void) {
    return 1000 + rand() % 9000;
}

/* Gives the customer up to OTP_MAX_TRIES attempts to enter the correct
 * OTP to confirm the package was actually received. Returns 1 on
 * success, 0 if verification failed. */
static int verifyOTP(int correctOTP) {
    for (int attempt = 1; attempt <= OTP_MAX_TRIES; attempt++) {
        int entered = readInt("Enter the 4-digit OTP given by the courier to confirm receipt: ");
        if (entered == correctOTP) {
            printf("OTP verified successfully. Delivery confirmed as RECEIVED.\n");
            return 1;
        }
        printf("Incorrect OTP (attempt %d of %d).\n", attempt, OTP_MAX_TRIES);
    }
    printf("Too many incorrect attempts. Delivery could NOT be verified. "
           "Please contact support.\n");
    return 0;
}

/* -------------------------------------------------------------------
 *  ESTIMATE & RECEIVE DELIVERY
 *  - Uses Dijkstra to find the shortest distance from the Hub straight
 *    to the customer's receiving point (not the multi-stop route).
 *  - Shows the total cost the customer must pay and the estimated
 *    wait time before the courier arrives.
 *  - Requires the customer to be logged in.
 *  - Finishes with 4-digit OTP verification to confirm receipt.
 * ------------------------------------------------------------------- */

static void receiveDelivery(Graph *g) {
    if (loggedInUserIdx == -1) {
        printf("Please log in or register an account first (menu option 9 or 10).\n");
        return;
    }
    if (g->numLoc < 2) {
        printf("No delivery points have been added yet.\n");
        return;
    }

    int dest = -1;
    int fav = users[loggedInUserIdx].favLocation;

    if (fav != -1 && fav < g->numLoc) {
        char useFav[10];
        printf("\nYour favorite receiving point is '%s'.\n", g->names[fav]);
        readName("Deliver to your favorite point? (yes/no): ", useFav, sizeof(useFav));
        if (strcmp(useFav, "yes") == 0 || strcmp(useFav, "y") == 0 ||
            strcmp(useFav, "Yes") == 0 || strcmp(useFav, "Y") == 0) {
            dest = fav;
        }
    }

    if (dest == -1) {
        printf("\nAvailable receiving points: ");
        for (int i = 1; i < g->numLoc; i++) printf("[%d]%s ", i, g->names[i]);
        printf("\n");
        dest = readInt("Enter the index of your receiving location: ");
    }

    if (dest <= 0 || dest >= g->numLoc) {
        printf("Error: invalid location index.\n");
        return;
    }

    printf("\nYou selected: %s\n", g->names[dest]);

    double dist[MAX_LOC];
    int parent[MAX_LOC];
    dijkstra(g, 0, dist, parent);   /* shortest distance Hub -> every point */

    if (dist[dest] >= INF) {
        printf("Sorry, '%s' is currently unreachable from the Hub "
               "(no road path exists yet).\n", g->names[dest]);
        return;
    }

    double distance   = dist[dest];
    double cost        = distance * g->fuelRate;
    double timeHours   = distance / AVG_SPEED_KMH;
    int    hours        = (int) timeHours;
    int    minutes       = (int) ((timeHours - hours) * 60.0 + 0.5);
    if (minutes == 60) { minutes = 0; hours++; }

    printf("\n--- Delivery Estimate for %s ---\n", users[loggedInUserIdx].name);
    printf("Destination        : %s\n", g->names[dest]);
    printf("Shortest Distance   : %.2f km   [Path: ", distance);
    printPath(g, parent, dest);
    printf("]\n");
    printf("Estimated Wait Time : %d hr %d min (at %.0f km/h)\n", hours, minutes, AVG_SPEED_KMH);
    printf("Total Amount to Pay : Tk. %.2f\n", cost);

    char confirm[10];
    readName("Confirm payment and proceed with delivery? (yes/no): ", confirm, sizeof(confirm));
    if (strcmp(confirm, "yes") != 0 && strcmp(confirm, "y") != 0 &&
        strcmp(confirm, "Yes") != 0 && strcmp(confirm, "Y") != 0) {
        printf("Delivery cancelled.\n");
        return;
    }

    printf("\nPayment of Tk. %.2f received. Your package is on its way to %s.\n",
           cost, g->names[dest]);
    printf("Please wait approximately %d hr %d min.\n", hours, minutes);

    int otp = generateOTP();
    printf("\n[SIMULATED SMS to %s] Your delivery OTP is: %04d\n", users[loggedInUserIdx].phone, otp);
    printf("(In production this would be sent by real SMS/notification instead of printed here.)\n");

    printf("\nWhen the courier hands over the package, enter the OTP to confirm receipt.\n");
    verifyOTP(otp);
}

/* -------------------------------------------------------------------
 *  MAIN
 * ------------------------------------------------------------------- */

int main(void) {
    Graph g;
    initGraph(&g);
    seedDefaultNetwork(&g);
    srand((unsigned) time(NULL));

    printf("===================================================\n");
    printf("  Default network loaded:\n");
    printf("  Hub = DIU (Daffodil International University, Ashulia)\n");
    printf("  Points = Ashulia, Tongi, Uttara, Mirpur, Dhamrai,\n");
    printf("           Banani, Gulshan, Dhanmondi\n");
    printf("  (Road distances are approximate real-road driving\n");
    printf("   distances; edit anytime with menu options 1 & 2.)\n");
    printf("===================================================\n");

    int choice;
    do {
        printf("\n===================================================\n");
        printf("   SMART COURIER & DELIVERY HUB OPTIMIZER\n");
        printf("   (Dijkstra's Algorithm + Greedy Nearest Neighbor)\n");
        printf("===================================================\n");
        if (loggedInUserIdx != -1)
            printf("   Logged in as: %s\n", users[loggedInUserIdx].name);
        else
            printf("   Not logged in\n");
        printf("---------------------------------------------------\n");
        printf("1. Add Delivery Location\n");
        printf("2. Add Road Connection\n");
        printf("3. View Road Network\n");
        printf("4. Shortest Paths from Hub (Dijkstra)\n");
        printf("5. Optimize Delivery Route (Greedy Nearest Neighbor)\n");
        printf("6. Compare Optimized vs Random Routes\n");
        printf("7. Set Fuel Cost Rate (currently Tk. %.2f/km)\n", g.fuelRate);
        printf("8. Account Status\n");
        printf("9. Register Account (name, phone, age)\n");
        printf("10. Login\n");
        printf("11. Logout\n");
        printf("12. Set Favorite Receiving Point\n");
        printf("13. Estimate & Receive Delivery (Hub -> your point, pay, OTP verify)\n");
        printf("14. Exit\n");
        printf("---------------------------------------------------\n");
        choice = readInt("Enter choice: ");

        switch (choice) {

        case 1: {
            char name[NAME_LEN];
            readName("Enter delivery location name (no spaces): ", name, NAME_LEN);
            if (strlen(name) == 0) { printf("Error: name cannot be empty.\n"); break; }
            int idx = addLocation(&g, name);
            if (idx == -1) printf("Error: maximum locations (%d) reached.\n", MAX_LOC);
            else if (idx == -2) printf("Error: a location named '%s' already exists.\n", name);
            else printf("Added '%s' as location [%d].\n", name, idx);
            break;
        }

        case 2: {
            if (g.numLoc < 2) { printf("Add at least one delivery location first.\n"); break; }
            printf("Locations: ");
            for (int i = 0; i < g.numLoc; i++) printf("[%d]%s ", i, g.names[i]);
            printf("\n");
            int u = readInt("Enter first location index: ");
            int v = readInt("Enter second location index: ");
            double d = readDouble("Enter road distance (km): ");
            int res = addRoad(&g, u, v, d);
            if (res == -1) printf("Error: invalid location index.\n");
            else if (res == -2) printf("Error: cannot connect a location to itself.\n");
            else if (res == -3) printf("Error: distance must be positive.\n");
            else if (res == 1) printf("Road updated: %s <-> %s = %.2f km\n", g.names[u], g.names[v], d);
            else printf("Road added: %s <-> %s = %.2f km\n", g.names[u], g.names[v], d);
            break;
        }

        case 3:
            viewNetwork(&g);
            break;

        case 4:
            showShortestPathsFromHub(&g);
            break;

        case 5: {
            if (g.numLoc < 2) { printf("Add delivery points first.\n"); break; }
            double distMat[MAX_LOC][MAX_LOC];
            computeAllPairs(&g, distMat);
            RouteResult r = greedyNearestNeighbor(&g, distMat);
            printf("\n--- Optimized Delivery Route (Greedy Nearest Neighbor) ---\n");
            printRoute(&g, &r);
            break;
        }

        case 6:
            compareRoutes(&g);
            break;

        case 7: {
            double rate = readDouble("Enter new fuel cost per km (Tk.): ");
            if (rate <= 0) { printf("Error: fuel rate must be positive.\n"); break; }
            g.fuelRate = rate;
            printf("Fuel rate updated to Tk. %.2f/km\n", rate);
            break;
        }

        case 8:
            showAccountStatus(&g);
            break;

        case 9:
            registerUser();
            break;

        case 10:
            loginUser();
            break;

        case 11:
            logoutUser();
            break;

        case 12:
            setFavoriteLocation(&g);
            break;

        case 13:
            receiveDelivery(&g);
            break;

        case 14:
            printf("Exiting. Safe deliveries!\n");
            break;

        default:
            printf("Invalid choice, try again.\n");
        }

    } while (choice != 14);

    return 0;
}

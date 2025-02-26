#include "encoder.h"
#include "motor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "IR.h"
#include <queue>

const int dx[] = {0, -1, 0, 1};
const int dy[] = {1, 0, -1, 0};

#define ONE_CELL 14.3
#define DIR_UP 0
#define DIR_LEFT 1
#define DIR_DOWN 2
#define DIR_RIGHT 3

#define LOOK_FORWARD 0
#define LOOK_LEFT 1
#define LOOK_BACKWARD 2
#define LOOK_RIGHT 3
#define STOP_MOUSE 4

#define DEST_CENTER 0
#define DEST_SOURCE 1

#define MAZESIDE_SIZE 6

#define START_DIR DIR_UP
#define x_end 4
#define y_end 4
#define INF 99
typedef unsigned char u8;
typedef unsigned int u32;
#define dir_change(x, y) (x + y) % 4



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if (!bno.begin())
  {
      while (1);
  }
  delay(500);
  bno.setExtCrystalUse(true);
  
  pinMode(ir_tx1, OUTPUT);
  pinMode(ir_tx2, OUTPUT);
  pinMode(ir_tx3, OUTPUT);
  pinMode(ir_rx1, INPUT);
  pinMode(ir_rx2, INPUT);
  pinMode(ir_rx3, INPUT);
  
  pinMode(En_LEFT_1, INPUT);
  pinMode(En_LEFT_2, INPUT);
  pinMode(En_RIGHT_1, INPUT);
  pinMode(En_RIGHT_2, INPUT);

  ledcSetup(PWM_CHANNEL_A1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_A2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_A1, PWM_CHANNEL_A1);
  ledcAttachPin(MOTOR_A2, PWM_CHANNEL_A2);
  ledcAttachPin(MOTOR_B1, PWM_CHANNEL_B1);
  ledcAttachPin(MOTOR_B2, PWM_CHANNEL_B2);

  bno.getEvent(&event);
  attachInterrupt(digitalPinToInterrupt(En_RIGHT_1), handleEncoderRight, RISING);  
  attachInterrupt(digitalPinToInterrupt(En_LEFT_1), handleEncoderLeft, RISING);

  //bnoCalibration();
}

u8 m_dir;
u8 m_x, m_y;
u8 m_destination;
std::queue<std::pair<u32, u32>> q, q_update;
struct MazeSquare
{
    bool blocked[4];
    u32 distance_to_dest;
    bool visited;
};

MazeSquare maze[MAZESIDE_SIZE][MAZESIDE_SIZE];


void initFloodfill()
{
    m_dir = START_DIR;
    m_destination = DEST_CENTER;
    m_x = 0;
    m_y = 0;
    for (int i = 0; i < MAZESIDE_SIZE; i++)
    {
        for (int j = 0; j < MAZESIDE_SIZE; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                maze[i][j].blocked[k] = 0;
            }
            maze[i][j].distance_to_dest = 0;
            maze[i][j].visited = 0;
        }
    }
    for (int i = 0; i < MAZESIDE_SIZE; i++)
    {
        maze[0][i].blocked[DIR_LEFT] = 1;
        maze[MAZESIDE_SIZE - 1][i].blocked[DIR_RIGHT] = 1;
        maze[i][MAZESIDE_SIZE - 1].blocked[DIR_UP] = 1;
        maze[i][0].blocked[DIR_DOWN] = 1;
    }
}
void floodfillToCenter()
{
    for (u8 i = 0; i < MAZESIDE_SIZE; i++)
    {
        for (u8 j = 0; j < MAZESIDE_SIZE; j++)
        {
            maze[i][j].distance_to_dest = INF;
        }
    }
    // for (u8 i = MAZESIDE_SIZE / 2 - 1; i <= MAZESIDE_SIZE / 2; i++)
    // {
    //     for (u8 j = MAZESIDE_SIZE / 2 - 1; j <= MAZESIDE_SIZE / 2; j++)
    //     {
    //         maze[i][j].distance_to_dest = 0;
    //         q.push(std::make_pair(i, j));
    //     }
    // }
    
    // Đặt ô mục tiêu
    q.push(std::make_pair(x_end, y_end));
    maze[x_end][y_end].distance_to_dest = 0;
    while (!q.empty())
    {
        auto u = q.front();
        q.pop();
        for (u8 i = 0; i < 4; i++)
        {
            if (!maze[u.first][u.second].blocked[i] && maze[u.first + dx[i]][u.second + dy[i]].distance_to_dest == INF)
            {
                maze[u.first + dx[i]][u.second + dy[i]].distance_to_dest = maze[u.first][u.second].distance_to_dest + 1;
                q.push(std::make_pair(u.first + dx[i], u.second + dy[i]));
            }
        }
    }
}

void floodfillToSource()
{
    for (u8 i = 0; i < MAZESIDE_SIZE; i++)
    {
        for (u8 j = 0; j < MAZESIDE_SIZE; j++)
        {
            maze[i][j].distance_to_dest = INF;
        }
    }
    maze[0][0].distance_to_dest = 0;
    q.push(std::make_pair(0, 0));
    while (!q.empty())
    {
        auto u = q.front();
        q.pop();
        for (u8 i = 0; i < 4; i++)
        {
            if (!maze[u.first][u.second].blocked[i] && maze[u.first + dx[i]][u.second + dy[i]].distance_to_dest == INF)
            {
                maze[u.first + dx[i]][u.second + dy[i]].distance_to_dest = maze[u.first][u.second].distance_to_dest + 1;
                q.push(std::make_pair(u.first + dx[i], u.second + dy[i]));
            }
        }
    }
}

bool checkFinish(u8 destination)
{
    // return (destination == DEST_CENTER 
    // && MAZESIDE_SIZE / 2 - 1 <= m_x && m_x <= MAZESIDE_SIZE / 2 
    // && MAZESIDE_SIZE / 2 - 1 <= m_y && m_y <= MAZESIDE_SIZE / 2) 
    // || (destination == DEST_SOURCE && m_x == 0 && m_y == 0);
    return (destination == DEST_CENTER && m_x == x_end && m_y == x_end);
}


void updateWalls(u8 m_view[4])
{
    for (int i = 0; i < 4; i++)
    {
        if (i == DIR_DOWN)
            continue;
        u8 new_dir = dir_change(m_dir, i);
        maze[m_x][m_y].blocked[new_dir] = m_view[i];
    }
    maze[m_x][m_y].visited = 1;
}

u8 minDistanceNeighbort(u8 m_x, u8 m_y)
{
    u8 min_dist = INF;
    for (int i = 0; i < 4; i++)
    {
        if (!maze[m_x][m_y].blocked[i] && maze[m_x + dx[i]][m_y + dy[i]].distance_to_dest < min_dist)
        {
            min_dist = maze[m_x + dx[i]][m_y + dy[i]].distance_to_dest;
        }
    }
    return min_dist;
}

u8 bestDir(u8 m_x, u8 m_y, u8 current_dist)
{
    u8 best_dir = LOOK_FORWARD;
    // ưu tiên đi thẳng nếu các ô cung quanh ô hiện tại có cùng giá trị
    for (int i = 3; i >= 0; i--)
    {
        u8 new_dir = dir_change(m_dir, i);
        if (!maze[m_x][m_y].blocked[new_dir] && maze[m_x + dx[new_dir]][m_y + dy[new_dir]].distance_to_dest < current_dist)
        {
            best_dir = i;
        }
    }
    return best_dir;
}
void floodfill(u8 x, u8 y)
{
    q_update.push(std::make_pair(x, y));
    while (!q_update.empty())
    {
        auto u = q_update.front();
        q_update.pop();
        u8 minimunValue = minDistanceNeighbort(u.first, u.second);
        if (maze[u.first][u.second].distance_to_dest <= minimunValue)
        {
            maze[u.first][u.second].distance_to_dest = minimunValue + 1;
            for (int i = 0; i < 4; i++)
            {
                if (!maze[u.first][u.second].blocked[i])
                {
                    q_update.push(std::make_pair(u.first + dx[i], u.second + dy[i]));
                }
            }
        }
    }
}
u8 getNextCommand(u8 forward_blocked, u8 left_blocked, u8 right_blocked)
{
    // mặc định hướng bên dưới mouse = 0
    u8 view_blocked[4] = {forward_blocked, left_blocked, 0, right_blocked};
    if (checkFinish(m_destination))
    {
        return STOP_MOUSE;
    }
    if (!maze[m_x][m_y].visited)
    {
        updateWalls(view_blocked);
    }

    u8 best_dist = minDistanceNeighbort(m_x, m_y);
    if (best_dist >= maze[m_x][m_y].distance_to_dest)
    {
        floodfill(m_x, m_y);
    }
    u8 best_dir = bestDir(m_x, m_y, maze[m_x][m_y].distance_to_dest);
    return best_dir;
}
void goForward()
{
    m_x += dx[m_dir];
    m_y += dy[m_dir];
}
void control()
{
    u8 forward_blocked = wallFront();
    u8 left_blocked = wallLeft();
    u8 right_blocked = wallRight();

    u8 command = getNextCommand(forward_blocked, left_blocked, right_blocked);

    if (command == 0)
    {
        goStraight(ONE_CELL);
    }
    if (command == 1)
    {
        turnLeft(90);
        goStraight(ONE_CELL);
    }
    if (command == 2)
    {
        turnLeft(90);
        turnLeft(90);
        goStraight(ONE_CELL);
    }
    if (command == 3)
    {
        turnRight(90);
        goStraight(ONE_CELL);
    }
    if (command == 4)
    {
        motorStop();
        delay(10000);
    }
    m_dir = dir_change(m_dir, command);
    goForward();
}
void loop()
{
    initFloodfill();
    floodfillToCenter();
    while (true)
    {
        control();
    }
}

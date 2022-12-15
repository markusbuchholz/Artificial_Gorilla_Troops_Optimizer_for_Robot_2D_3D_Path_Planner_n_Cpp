// Markus Buchholz
// g++ artificial_gorilla_troops_robot2D.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//plot
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


//--------Path Planner--------------------------------------------------------------

float xmin = 0.0;
float xmax = 50.0;
float ymin = 0.0;
float ymax = 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 3.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

float K1 = 0.15;                 //
float K2 = 0.000000000000000000000001; //

//--------------------------------------------------------------------------------
int DIM = 2;
int EVOLUTIONS = 1;
int GORILLAS = 150;
float P = 0.05;
float W = 0.8;
float B = 3;
//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

float generateRandomX()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//--------------------------------------------------------------------------------

float generateRandomH_E(float C)
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-C, C);
    return distribution(gen);
}
//--------------------------------------------------------------------------------
float generateRandomNormalDist()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::normal_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{

    std::vector<float> funcValue;
    Pos Obs{obsX, obsY};
    Pos Goal{goalX, goalY};

    for (auto &ii : pos)
    {

        funcValue.push_back(K1 * (1 / euclid(Obs, ii)) + K2 * euclid(Goal, ii));
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{
    Pos Obs{obsX, obsY};
    Pos Goal{goalX, goalY};

    return K1 * (1 / euclid(Obs, pos)) + K2 * euclid(Goal, pos);
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    return Pnew;
}
//--------------------------------------------------------------------------------

Pos newPosXY()
{

    Pos pos = {valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)};

    // return positionUpdateCheck(pos);
    return pos;
}

//--------------------------------------------------------------------------------

Pos posNew(Pos gorilla, Pos memeber, float C)
{

    float L = C * generateRandomX();
    Pos Xnew;

    float rand1 = generateRandom();

    // migrate to unknow place
    if (rand1 < P)
    {

        Xnew = newPosXY();
    }
    // migrate to other troops(group)
    else if (rand1 >= P)
    {

        Xnew.x = (generateRandom() - C) * memeber.x + L * generateRandomH_E(C);
        Xnew.y = (generateRandom() - C) * memeber.y + L * generateRandomH_E(C);
    }

    // migrate to know place
    else if ((rand1 > 0.5) && (rand1 > P))
    {

        Xnew.x = gorilla.x - L * (L * (gorilla.x - memeber.x)) + generateRandom() * (gorilla.x - memeber.x);
        Xnew.y = gorilla.y - L * (L * (gorilla.y - memeber.y)) + generateRandom() * (gorilla.y - memeber.y);
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

Pos posNewExp(std::vector<Pos> currentPositions, Pos gorilla, Pos silver, float C)
{

    float L = C * generateRandomX();
    Pos Xnew;

    if (std::abs(C) >= 1)
    {

        float g = std::pow(2, L);
        float Mxi = 0;
        float Myi = 0;

        int N = currentPositions.size();

        for (auto &ii : currentPositions)
        {

            Mxi += ii.x;
            Myi += ii.y;
        }

        float Mx = std::pow(std::pow(std::abs((1 / N) * Mxi), g), 1 / g);
        float My = std::pow(std::pow(std::abs((1 / N) * Myi), g), 1 / g);

        Xnew.x = L * Mx * (gorilla.x - silver.x) + gorilla.x;
        Xnew.y = L * My * (gorilla.y - silver.y) + gorilla.y;
    }

    else
    {

        float Q = 2 * generateRandom() - 1;

        float E = generateRandom() >= 0.5 ? generateRandomNormalDist() : generateRandomH_E(DIM);

        Xnew.x = silver.x - (silver.x * Q - gorilla.x * Q) * B * E;
        Xnew.y = silver.y - (silver.y * Q - gorilla.y * Q) * B * E;
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < GORILLAS; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}

//-------------------------------------------------------------------------------
bool compareMax(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second > b.second;
}

//-------------------------------------------------------------------------------

// max
std::tuple<Pos, float> findWorstPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMax);

    return best[0];
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<Pos, float> findBestPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    return best[0];
}

//-------------------------------------------------------------------------------

int chooseMember(int actual)
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, GORILLAS);

    int r = -1;

    do
    {

        r = distribution(engine);

    } while (r == actual);

    return r;
}

//-------------------------------------------------------------------------------

float computeC(int jj)
{

    float factor = 180 / M_PI;
    float F = std::cos(2 * generateRandom() * factor) + 1;

    return F * (1 - jj / EVOLUTIONS);
}

//-------------------------------------------------------------------------------

std::vector<Pos> runGTO()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> currentValueFunction = function(currentPositions);

    for (int jj = 0; jj < EVOLUTIONS; jj++)
    {

        float C = computeC(jj);

        // Exploration
        for (int ii = 0; ii < GORILLAS; ii++)
        {

            int member = chooseMember(ii);
            Pos newPos = posNew(currentPositions[ii], currentPositions[member], C);
            float newValueFunc = func(newPos);
            if (newValueFunc < currentValueFunction[ii])
            {

                currentPositions[ii] = newPos;
                currentValueFunction[ii] = newValueFunc;
            }
        }

        // Explotation

        auto best = findBestPosFuncValue(currentPositions, currentValueFunction);
        auto bestPos = std::get<0>(best);
        auto bestValue = std::get<0>(best);

        for (int ii = 0; ii < GORILLAS; ii++)
        {
            Pos newPos = posNewExp(currentPositions, currentPositions[ii], bestPos, C);
            float newValueFunc = func(newPos);
            if (newValueFunc < currentValueFunction[ii])
            {

                currentPositions[ii] = newPos;
                currentValueFunction[ii] = newValueFunc;
            }
        }
    }


    return currentPositions;
}
//-------------------------------------------------------------------------------
std::tuple<std::vector<float>, std::vector<float>> gen_circle(float a, float b, float r)
{

    std::vector<float> xX;
    std::vector<float> yY;

    for (float dt = -M_PI; dt < M_PI; dt += 0.01)
    {

        xX.push_back(a + r * std::cos(dt));
        yY.push_back(b + r * std::sin(dt));
    }
    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());

    std::tuple<std::vector<float>, std::vector<float>> circle = gen_circle(obsX, obsY, obsR);

    std::vector<float> xObs = std::get<0>(circle);
    std::vector<float> yObs = std::get<1>(circle);

    plt::plot(xX, yY);
    plt::plot(xObs, yObs);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}

//-------------------------------------------------------------------------------

int main()
{

    
    std::vector<Pos> path = runGTO();

    std::vector<float> xX;
    std::vector<float> yY;

    for (auto &ii : path)
    {
        xX.push_back(ii.x);
        yY.push_back(ii.y);

        std::cout << ii.x << " ," << ii.y << "\n";
    }

    plot2D(xX, yY);
}

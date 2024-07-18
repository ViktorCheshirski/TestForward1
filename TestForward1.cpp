#include <iostream>
#include <vector>

using namespace std;

class Engine {
public:
    double inertia; // Момент инерции
    vector<double> torque; // Крутящий момент
    vector<double> speed; // Скорость вращения
    double overheatTemp; // Температура перегрева
    double heatingCoeffTorque; // Коэффициент нагрева от момента
    double heatingCoeffSpeed; // Коэффициент нагрева от скорости
    double coolingCoeff; // Коэффициент охлаждения

    Engine(double inertia, vector<double> torque, vector<double> speed,
        double overheatTemp, double heatingCoeffTorque, double heatingCoeffSpeed, double coolingCoeff)
        : inertia(inertia), torque(torque), speed(speed), overheatTemp(overheatTemp),
        heatingCoeffTorque(heatingCoeffTorque), heatingCoeffSpeed(heatingCoeffSpeed), coolingCoeff(coolingCoeff) {}
};

class EngineSimulation {
public:
    int simulateTimeToOverheat(Engine& engine, double ambientTemp) {
        double engineTemp = ambientTemp;
        int currentTime = 0;
        double currentSpeed = 0;

        while (engineTemp < engine.overheatTemp) {
            double torque = interpolateTorque(engine, currentSpeed);
            double acceleration = torque / engine.inertia;
            currentSpeed += acceleration;

            double heatingRate = torque * engine.heatingCoeffTorque + pow(currentSpeed, 2) * engine.heatingCoeffSpeed;
            double coolingRate = engine.coolingCoeff * (ambientTemp - engineTemp);

            engineTemp += heatingRate - coolingRate;
            currentTime += 1;
        }

        return currentTime;
    }

    vector<double>  simulateMaxPower(Engine& engine) {
        double currentSpeed = 0;
        double maxPower = 0;
        double speedAtMaxPower = 0;

        while (true) {
            double torque = interpolateTorque(engine, currentSpeed);
            double acceleration = torque / engine.inertia;
            currentSpeed += acceleration;

            double power = (torque * currentSpeed) / 1000; // Мощность в кВт
            if (power > maxPower) {
                maxPower = power;
                speedAtMaxPower = currentSpeed;
            }
            else break;
        }

        return { maxPower, speedAtMaxPower };
    }

private:
    double interpolateTorque(Engine& engine, double currentSpeed) {
        for (size_t i = 1; i < engine.speed.size(); ++i) {
            if (currentSpeed < engine.speed[i]) {
                double v1 = engine.speed[i - 1];
                double v2 = engine.speed[i];
                double m1 = engine.torque[i - 1];
                double m2 = engine.torque[i];
                return m1 + (m2 - m1) * (currentSpeed - v1) / (v2 - v1);
            }
        }
        return 0;
    }
};

int main() {
    setlocale(LC_ALL, "Russian");
    // Инициализация параметров двигателя
    double inertia = 10.0;
    vector<double> torque = { 20, 75, 100, 105, 75, 0 };
    vector<double> speed = { 0, 75, 150, 200, 250, 300 };
    double overheatTemp = 110;
    double heatingCoeffTorque = 0.01;
    double heatingCoeffSpeed = 0.0001;
    double coolingCoeff = 0.1;

    Engine engine(inertia, torque, speed, overheatTemp, heatingCoeffTorque, heatingCoeffSpeed, coolingCoeff);

    // Ввод температуры окружающей среды
    double ambientTemp;
    cout << "Введите температуру окружающей среды (в градусах Цельсия): ";
    cin >> ambientTemp;

    EngineSimulation simulation;

    // Тест 1: Время до перегрева
    double timeToOverheat = simulation.simulateTimeToOverheat(engine, ambientTemp);
    cout << "Время до перегрева: " << timeToOverheat << " секунд" << endl;

    // Тест 2: Максимальная мощность
    vector<double>  result = simulation.simulateMaxPower(engine);
    cout << "Максимальная мощность: " << result[0] << " кВт" << endl;
    cout << "Скорость вращения коленвала при максимальной мощности: " << result[1] << " радиан/сек" << endl;

    return 0;
}

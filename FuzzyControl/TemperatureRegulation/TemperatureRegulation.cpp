//File: TemperatureRegulation.cpp
#include "fl/Headers.h"

int main(int argc, char* argv[]){
    using namespace fl;
    Engine* engine = FllImporter().fromFile("TemperatureRegulation.fll");

    std::string status;
    if (not engine->isReady(&status))
        throw Exception("[engine error] engine is not ready:n" + status, FL_AT);

    InputVariable* temperature = engine->getInputVariable("temperature");
    InputVariable* moisture = engine->getInputVariable("moisture");
    OutputVariable* tempChange = engine->getOutputVariable("tempChange");

    temperature->setValue(19.5);
    moisture->setValue(65);

    engine->process();
    FL_LOG(" temperature.input = " << Op::str(19.5) << 
    " moisture.input = " << Op::str(65) << 
        " => " << "Temperature Change.output = " << Op::str(tempChange->getValue()));


 }

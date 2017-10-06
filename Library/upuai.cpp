/* -------------------------------------------------------------------------- *
 *                    upamanyuai : upuai.cpp                                  *
 * -------------------------------------------------------------------------- *
 * The following file is a changed version file from the library on simbody   *
 * and opensim engine .                                                        *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 *                                                                            *
 *            Changes has been made to optimize the fuctions                  *
 * -------------------------------------------------------------------------- */

#include <upmai/upmai.h>
#include "upmai/Common/STOFileAdapter.h"
#include <ctime> 
#include <OpenSim/OpenSim.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include <ctime>

using namespace upmai;
using namespace uptk;
using namespace std;

int stepCount = 0; 
const double initialTime = 0.0;
const double finalTime = 0.25;
const double desired_accuracy = 1.0e-5;
double bestSoFar = Infinity;

class controlsys : public OptimizerSystem {
public:

    controlsys(int numParameters, State& s, Model& aModel): 
        OptimizerSystem(numParameters), 
        numControls(numParameters), 
        si(s),
        upuaim(aModel)
    {
        
        p_integrator = new RungeKuttaMersonIntegrator(upuaim.getMultibodySystem());
        p_integrator->setAccuracy(desired_accuracy);
    }
                
    int objectiveFunc(const Vector &newControls,
        bool new_coefficients, Real& f) const override {

        State s = si;

        upuaim.updDefaultControls() = newControls;
            
        Manager manager(upuaim, *p_integrator);
        s.setTime(initialTime);

        upuaim.getMultibodySystem().realize(s, Stage::Acceleration);

        manager.integrate(s, finalTime);

        const auto& hand = upuaim.getComponent<upmai::Body>("r_ulna_radius_hand");
        upuaim.getMultibodySystem().realize(s, Stage::Velocity);
        Vec3 massCenter = hand.getMassCenter();
        Vec3 velocity = hand.findStationVelocityInGround(s, massCenter);
        f = -velocity[0];
        stepCount++;
        
        if(f < bestSoFar) {
            bestSoFar = f;
            cout << "\nobjective evaluation #: " << stepCount << "  controls = "
                 << newControls <<  " bestSoFar = " << f << std::endl;
        }

      return 0;

   }    

private:
    int numControls;
    State& si;
    Model& upuaim;
    uptk::ReferencePtr<RungeKuttaMersonIntegrator> p_integrator;

 };

int main()
{
    try {
        std::clock_t startTime = std::clock();  

        Object::renameType("Thelen2003Muscle", "Millard2012EquilibriumMuscle");
   
        Model upuaim("Arm26_Optimize.upu");

        State& si = upuaim.initSystem();

        const CoordinateSet& coords = upuaim.getCoordinateSet();
        coords.get("r_shoulder_elev").setValue(si, -1.57079633);

        const Set<Muscle> &muscleSet = upuaim.getMuscles();
        for(int i=0; i<muscleSet.getSize(); ++i) {
            muscleSet[i].setActivation(si, 0.01);
            muscleSet[i].setIgnoreTendonCompliance(si, true);
        }
    
        upuaim.equilibrateMuscles(si);

        int numControls = upuaim.getNumControls();
        
        controlsys sys(numControls, si, upuaim);
        Real f = NaN;
        
        Vector controls(numControls, 0.02);
        controls[3] = 0.99;
        controls[4] = 0.99;
        controls[5] = 0.99;

        Vector lower_bounds(numControls, 0.01);
        Vector upper_bounds(numControls, 0.99);

        sys.setParameterLimits( lower_bounds, upper_bounds );

        Optimizer opt(sys, uptk::LBFGSB);

        opt.setConvergenceTolerance(0.1);
        opt.useNumericalGradient(true, desired_accuracy);
        opt.setMaxIterations(2);
        opt.setLimitedMemoryHistory(500);

        f = opt.optimize(controls);
            
        cout << "Elapsed time = " << (std::clock()-startTime)/CLOCKS_PER_SEC << "s" << endl;
        
        const Set<Actuator>& actuators = upuaim.getActuators();
        for(int i=0; i<actuators.getSize(); ++i){
            cout << actuators[i].getName() << " control value = " << controls[i] << endl;
        }

        cout << "\nMaximum hand velocity = " << -f << "m/s" << endl;

        cout << "upmai example completed successfully." << endl;
        
        ofstream ofile; 
        ofile.open("Arm26_optimization_result"); 
        for(int i=0; i<actuators.getSize(); ++i)
            ofile << controls[i] << endl;
        ofile << -f <<endl;
        ofile.close(); 

        RungeKuttaMersonIntegrator integrator(upuaim.getMultibodySystem());
        integrator.setAccuracy(desired_accuracy);
        Manager manager(upuaim, integrator);
        upuaim.updDefaultControls() = controls;

        si.setTime(initialTime);
        upuaim.getMultibodySystem().realize(si, Stage::Acceleration);
        manager.integrate(si, finalTime);

        auto statesTable = manager.getStatesTable();
        STOFileAdapter_<double>::write(statesTable, 
                                      "Arm26_optimized_states.sto");
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    
    return 0;
}
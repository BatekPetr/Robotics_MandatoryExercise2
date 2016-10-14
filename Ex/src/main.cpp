#include <iostream>
// for converting numbers to strings
#include <sstream>
// for reading and writing from and to a file
#include <fstream>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

// includes for checking and creating directory
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 20.

//function prototypes
void findPath(Q from, Q to, QPath& path, QToQPlanner::Ptr planner, Timer& t, bool outMessages = true);

bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
	return true;
}

void createLuaFile(string name, QPath path)
{
    // Open given Lua file to copy first part
    std::ifstream  src("FilePathBeginning.txt");

    //Check Output folder in Build directory
    struct stat st = {0};

    if (stat("./Output", &st) == -1)
    {
        // If folder doesn't exist, create it
        mkdir("./Output", 0700);
    }

    // Create output stream
    std::ofstream fout("Output/" + name);
    if (!fout.is_open())
        cout << "Error opening file.\n";

    // Comment new file
    //todo

    // Copy first part from Luas file
    fout << src.rdbuf();
    src.close();


    for (QPath::iterator it = path.begin(); it < path.end(); it++) {
        std::vector<double> setQ = it->toStdVector();
        // Write to Lua file
        fout << "setQ({" << *setQ.begin();
        for (std::vector<double>::const_iterator i = setQ.begin() + 1; i != setQ.end(); ++i)
            fout << ", " << *i;
        fout << "})\n";
    }
    // Copy ending part of Lua script
    src.open("FilePathEnd.txt");
    fout << src.rdbuf();
    fout.close();
    src.close();
}

void writeToStatFile(std::ofstream& fout, double epsilon, unsigned long steps, Timer t)
{
    if (t.getTime() >= MAXTIME)
    {
        fout<< "NaN ,\t" << "NaN" << endl;
    }
    else
    {
        fout << epsilon << " ,\t" << steps << " ,\t" << t.getTime() << std::endl;
    }

}

void gripMovableFrame(MovableFrame& item, Frame& gripper, State& state)
{
    FKRange fk(&gripper, &item, state);
    const Transform3D<> transform = fk.get(state);
    item.setTransform(transform, state);
    item.attachTo(&gripper, state);
}

int main(int argc, char** argv) {
    // set Random seed
    rw::math::Math::seed();

    const string wcFile = "/media/petr/WD_HDD/SDU/RoVi1/Robotics/ManEx2/Kr16WallWorkCell/Scene.wc.xml";
    const string deviceName = "KukaKr16";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;

    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    // Starting position = position of bottle
    Q from(6, -3.142, -0.827, -3.002, -3.143, 0.099, -1.573);
    // End position for bottle
    Q to(6, 1.571, 0.006, 0.03, 0.153, 0.762, 4.49);



    // My code --------------------
    // Finds the frame of Gripper
    const string gripperName = "Tool";
    rw::kinematics::Frame *gripper = wc->findFrame(gripperName);
    // Finds the frame of Bottle
    const string itemName = "Bottle";
    rw::kinematics::MovableFrame *item;
    item = (MovableFrame *) wc->findFrame(itemName);
    if (gripper == NULL) {
        cerr << "Gripper: " << gripperName << " not found!" << endl;
        return 0;
    }
    if (item == NULL) {
        cerr << "Item: " << itemName << " not found!" << endl;
        return 0;
    }
    if (rw::kinematics::Kinematics::isDAF(item)) {
        cout << itemName << " is DAF frame. \n";
    }

    State state = wc->getDefaultState();
    device->setQ(from, state);
    Kinematics::gripFrame(item, gripper, state);
    // end of my code

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state);

    /** Most easy way: uses default parameters based on given device
        sampler: QSampler::makeUniform(device)
        metric: PlannerUtil::normalizingInfinityMetric(device->getBounds())
        extend: 0.1 */
    //QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, device, RRTPlanner::RRTConnect);

    /** More complex way: allows more detailed definition of parameters and methods */
    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device), constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

    // Set True for performation of statistical simulations
    // False - create Lua file for one epsilon parameter
    bool statistics = false;



    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    // planned path
    QPath path;
    Timer t;

    if (!statistics) {
        // set simulation epsilons - value in radians
        double extend = 0.25;
        QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

        findPath(from, to, path, planner, t);

        cout << "Path of length " << path.size() << " found in " << t.getTime() << " seconds." << endl;
        if (t.getTime() >= MAXTIME) {
            cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
        }
        for (QPath::iterator it = path.begin(); it < path.end(); it++) {
            cout << *it << endl;
        }

        // Create output Lua file
        // Convert double extend to string epsilon
        std::ostringstream strs;
        strs << extend;
        std::string epsilon = strs.str();

        //Output file name
        string fileName = "FilePath" + epsilon + ".txt";

        std::cout << fileName << std::endl;
        createLuaFile(fileName, path);
    }
    // Compute and create statistics file
    else
    {
        //Check Output folder in Build directory
        struct stat st = {0};

        if (stat("./Output", &st) == -1)
        {
            // If folder doesn't exist, create it
            mkdir("./Output", 0700);
        }

        // Create output stream and file
        string fileName = "stats.txt";
        std::ofstream fout("Output/" + fileName);
        if (!fout.is_open())
            cout << "Error opening file.\n";
        else
        {
            fout << "Epsilon , \t Path L ,\t Time[s]" << std::endl;

            // Create array of epsilon for statistical simulations
            double extend[] = {0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1}; // for statistics computation - values are in epsilon
            int extendLen = (sizeof(extend)/sizeof(*extend));
            for (int idx = 0; idx < extendLen; idx++)
            {
                // Do the simulations for one epsilon parametr
                double epsilon = extend[idx];

                // Set planner for given epsilon
                QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);
                std::cout << epsilon << std::endl; // Control console output

                // Do the simulations
                for (int i = 0; i < 1000; i++) {
                    path.clear();
                    findPath(from, to, path, planner, t, false);
                    writeToStatFile(fout, epsilon, path.size(), t);
                }
            }
            fout.close();

        }
    }
    cout << "Program done." << endl;
}

void findPath(Q from, Q to, QPath& path, QToQPlanner::Ptr planner, Timer& t, bool outMessages)
{
    if(outMessages)
	    cout << "Planning from " << from << " to " << to << endl;

	t.resetAndResume();
	planner->query(from,to,path,MAXTIME);
	t.pause();

}

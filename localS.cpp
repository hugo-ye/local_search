#include <localS.h>
#include <iostream>
#include <sstream>
#include <string>

using namespace localS;


void LsProblemManager::label (naxos::NsIntVarArray& varArray_, Configuration* conf_)
{
	varArray 	= &varArray_;
	conf 		= conf_;

	varArray->lsLabeling();
}


std::ostream& LsProblemManager::solutionToString (std::ostream& out)
{
	out << "\nSolution: [";
	for ( naxos::NsIndex i = 0, size = varArray->size() ; i < size ; i++ )
	{
		out << "[" << (*varArray)[i].lsValue() << "]";
		if ( i != size - 1 ) out << " ";
	}
	out << "]" << std::endl;

	return out;
}


void LsProblemManager::nextSolution (void)
{
	naxos::assert_Ns( varArray != NULL , "LsProblemManager::nextSolution: You must first call `LsProblemManager::label'" );

	Timer timer;
	std::string hash;
	while ( true )
	{
		// Initialize at the beginning of each search process
		globalMinConflicts = -1;

		if ( !previousSolutions.empty() ) reset();

		timer.start(); 				// Start timing
		if 	( conf->algorithm() == HILL ) 		solveHill();
		else if ( conf->algorithm() == ANNEALING ) 	solveAnnealing();
		elapsedTime = timer.elapsed(); 		// Get elapsed time

		hash = hashState(0, 0);
		// Current solution is a new one
		if ( previousSolutions.search( hash ) == 0 ) break;
		std::cerr << "Skipping solution with hash: " << hash << " (already found)" << std::endl;
	}
	previousSolutions.push( hash );
}


void LsProblemManager::solveHill (void)
{
	using namespace std;
	using namespace naxos;

	initialize();

	// Force a random walk
	bool 			doRandom = false;
	// Attempts to avoid restart
	unsigned long 	attempts = 0;
	// Minimum number of conflicting constraints so far
	unsigned long 	minConflicts = lsViolatedConstraints().size();
	// For the random walks
	RandomVariable 	randomVariable( *this );
	RandomValue 	randomValue( *this );
	// Keep the previous states (hash) with the same number of conflicting constraints
	ActiveWindow<std::string> previousStates;

	HillConfiguration* hConf = static_cast<HillConfiguration*> (conf);
	hConf->steps = 0; hConf->maxSteps = 0; hConf->restarts = 0;
	// Repeat till a solution is found
	while ( !lsViolatedConstraints().empty() )
	{
		// Select a Variable and a Value for that Variable

		VariablePtr selectedVariablePtr;

		// With walkProb probability perform a random walk
		if ( (random( 1000 ) / 1000.0) < hConf->walkProb || doRandom )
		{
			//std::cerr << "\t(Random Walk...)" << std::endl;
			selectedVariablePtr = randomVariable.select();
			randomValue.select( *selectedVariablePtr );
			doRandom = false;
		}
		// Else (with probability 1-walkProb) perform a standard step
		else
		{
			//std::cerr << "\t(Standard Walk...)" << std::endl;
			selectedVariablePtr = hConf->variableHeuristic->select();
			hConf->valueHeuristic->select( *selectedVariablePtr );
		}

		// Keep in previousStates only states with the same number of conflicting constraints
		if ( lsViolatedConstraints().size() < minConflicts )
		{
			minConflicts = lsViolatedConstraints().size();
			previousStates.clear();
		}

		string stateHash = hashState(selectedVariablePtr->lsIndex(), selectedVariablePtr->lsValue());
		//std::cerr << "Conflicts: " << lsViolatedConstraints().size() << " | Var: " << selectedVariablePtr->lsIndex();
		//std::cerr << " --> Value: " <<  selectedVariablePtr->lsValue() << " | " << stateHash << " || " << previousStates.size() << std::endl;
		previousStates.push( stateHash );

		hConf->steps++;
		// States in active window repeat themselves; restart the process
		if ( previousStates.search( stateHash ) >= hConf->maxStateRepeats )
		{
			if ( attempts++ == hConf->maxAvoidAttempts )
			{
				//cerr << "Restarting...\n" << endl;
				hConf->restarts++;
				hConf->maxSteps = (hConf->maxSteps < hConf->steps ? hConf->steps : hConf->maxSteps);
				hConf->steps = 0;
				doRandom = false;
				attempts = 0;
				previousStates.clear();

				reset();
				initialize();
			}
			else
			{
				//cerr << "Avoiding restart...\n" << endl;
				// Next step should be a random walk in hope that it will avoid the repetition of states
				doRandom = true;
				previousStates.clear();
			}
		}
	}
	// Update maxSteps
	if ( hConf->steps > hConf->maxSteps ) hConf->maxSteps = hConf->steps;
	// If no restart occured
	if ( hConf->maxSteps == 0 ) hConf->maxSteps = hConf->steps;
}


void LsProblemManager::solveAnnealing (void)
{
	using namespace std;
	using namespace naxos;

	// For the random walks
	RandomVariable 	randomVariable( *this );
	RandomValue 	randomValue( *this );

	initialize();

	AnnealingConfiguration* aConf = static_cast<AnnealingConfiguration*> (conf);

	double T = 0.0;
	unsigned long stableSteps = aConf->scheduler->stablePeriod();
	aConf->steps = 0; aConf->restarts = 0;
	for (unsigned long t = 0, k = 0 ; t < naxos::NsUPLUS_INF ; k = (k + 1) % stableSteps )
	{
		// The first time of the `stableSteps' repeats
		if ( k == 0 )
		{
			t++;
			// Get Temperature from scheduler for the current step
			T = (*aConf->scheduler)[t];
			// Restart process, if T reaches zero and no solution has been found
			if ( T <= 0 )
			{
				t = 1 ;
				aConf->restarts++;
				continue;
			}
		}

		// Select variable and value at random
		VariablePtr selectedVariablePtr = randomVariable.select();

		NsInt 	currentValue = selectedVariablePtr->lsValue();
		int 	currentConflicts = lsViolatedConstraints().size();

		NsInt 	selectedValue = randomValue.select( *selectedVariablePtr );
		int 	nextConflicts = lsViolatedConstraints().size();
		
		int 	de = nextConflicts - currentConflicts;

		// Same value selected
		if ( currentValue == selectedValue ) continue;

		// Found a solution
		if ( nextConflicts == 0 )
		{
			// Record total steps
			aConf->steps = (t - 1) * stableSteps + (k + 1) ;
			break;
		}

		// Current move is an improvement; accept it
		if ( de <= 0 ) continue;

		// Else accept current move with probability e^(de/T)

		// Generates double floating point numbers in the closed interval [0, 1]
		double threshold = static_cast<double>(random()) * (1. / 4294967295.); // divided by 2^32 - 1
		double decay = exponentialDecay( -de / (T * 1.0) );

		// Acceptance with probability e^(de/T) means threshold > decay; otherwise undo the last assignment
		if ( threshold >= decay )
		{
			revertToAssignment( make_pair(selectedVariablePtr, currentValue) );
		}
	}
}



////////////////////////////////////// Various Statistics //////////////////////////////////////


std::ostream& LsProblemManager::configuration (std::ostream& out)
{
	out << std::endl;
	out << "------------------------------------------------------" << std::endl;
	out << "---------------------Configuration--------------------" << std::endl;
	out << "------------------------------------------------------" << std::endl;
	out << "Random generator running with seed: `" << seed << "'" << std::endl;
	out << "Tabu Tenure: `" << tabuTenure << "' states" << std::endl;
	// Print configuration parameters specific for the algorithm used
	conf->configuration(out);
	out << "------------------------------------------------------" << std::endl;

	return out;
}

std::ostream& LsProblemManager::statistics (std::ostream& out)
{
	out << std::endl;
	out << "------------------------------------------------------" << std::endl;
	out << "-----------------Solution  Statistics-----------------" << std::endl;
	out << "------------------------------------------------------" << std::endl;
	out << "Elapsed time: `" << elapsedTime << "' sec" << std::endl;
	// Print statistics specific for the algorithm used
	conf->statistics(out);
	out << "------------------------------------------------------" << std::endl;

	return out;
}


std::ostream& LsProblemManager::HillConfiguration::configuration (std::ostream& out)
{
	out << "Algorithm used: Hill Climbing" << std::endl;
	out << "Active Window allowing state repetitions: `" << maxStateRepeats << "' times" << std::endl;
	out << "Max Attempts to avoid restart: `" << maxAvoidAttempts << "' times" << std:: endl;
	out << "Walking Probability: `" << walkProb << "'" << std::endl;
	// Print configuration parameters specific for the heuristics used
	variableHeuristic->configuration(out);
	valueHeuristic->configuration(out);

	return out;
}

std::ostream& LsProblemManager::HillConfiguration::statistics (std::ostream& out)
{
	out << "Restarted: `" << restarts << "' times" << std::endl;
	out << "Max Steps ever reached: `" << maxSteps << "' steps" << std::endl;
	out << "In last iteration used: `" << steps << "' steps" << std::endl;

	return out;
}


std::ostream& LsProblemManager::AnnealingConfiguration::configuration (std::ostream& out)
{
	out << "Algorithm used: Simulated Annealing" << std::endl;
	// Print configuration parameters specific for the scheduler used
	scheduler->configuration(out);

	return out;
}

std::ostream& LsProblemManager::AnnealingConfiguration::statistics (std::ostream& out)
{
	out << "Restarted: `" << restarts << "' times" << std::endl;
	out << "In last iteration used: `" << steps << "' steps" << std::endl;

	return out;
}


std::ostream& TemperatureScheduler::configuration (std::ostream& out)
{
	out << "Keep Temperature stable for: `" << stableSteps << "' steps" << std::endl;

	return out;
}

std::ostream& LogarithmicScheduler::configuration (std::ostream& out)
{
	out << "Scheduler used: Logarithmic" << std::endl;
	out << "Parameter `d': `" << d << "'" << std::endl;
	
	TemperatureScheduler::configuration(out);

	return out;
}

std::ostream& GeometricScheduler::configuration (std::ostream& out)
{
	out << "Scheduler used: Geometric" << std::endl;
	out << "Parameter `r': `" << r << "'" << std::endl;
	
	TemperatureScheduler::configuration(out);

	return out;
}

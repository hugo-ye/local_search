#ifndef LOCAL_S_H
#define LOCAL_S_H

#include <naxos.h>
#include <mtrand.h>
#include <md5.h>
#include <auxiliary.h>

#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <cmath>


namespace localS
{


typedef naxos::NsIntVar* 				VariablePtr;
typedef std::pair<VariablePtr, naxos::NsInt> 		Assignment;

typedef naxos::NsList<naxos::NsIntVar*> 		ConfVariables;
typedef naxos::NsList<naxos::NsList<naxos::NsIntVar*> > ConfConstraints;
typedef ConfVariables::iterator 			ConfVarsIterator;
typedef ConfConstraints::iterator 			ConfConstrIterator;
typedef std::vector<ConfVarsIterator> 			ItVector;
typedef std::vector<naxos::NsInt> 			ValueVector;


class LsProblemManager;

////////////////////////////////////// VariableHeuristic //////////////////////////////////////

class VariableHeuristic
{

protected:

	LsProblemManager& pm;

public:

	VariableHeuristic (LsProblemManager& pm_) : pm(pm_) {}
	virtual ~VariableHeuristic(void) {}

	virtual VariablePtr select (void) = 0;
	
	virtual std::ostream& configuration (std::ostream& out) { return out; };
};


////////////////////////////////////// ValueHeuristic //////////////////////////////////////

class ValueHeuristic
{

protected:

	LsProblemManager& pm;

public:

	ValueHeuristic (LsProblemManager& pm_) : pm(pm_) {}
	virtual ~ValueHeuristic(void) {}

	virtual naxos::NsInt select (naxos::NsIntVar&) = 0;
	
	virtual std::ostream& configuration (std::ostream& out) { return out; };
};


////////////////////////////////////// ValueHeuristic //////////////////////////////////////

// Scheduler for the Simulated Annealing algorithm
class TemperatureScheduler
{

protected:

	LsProblemManager& pm;
	unsigned long stableSteps;

public:

	TemperatureScheduler (LsProblemManager& pm_, unsigned long stableSteps_ = 1) : pm(pm_), stableSteps(stableSteps_) {}
	virtual ~TemperatureScheduler(void) {}

	virtual double operator[] (unsigned long) = 0;
	
	unsigned long stablePeriod () { return stableSteps; }
	
	virtual std::ostream& configuration (std::ostream&);
};



////////////////////////////////////// LsProblemManager //////////////////////////////////////

class LsProblemManager : public naxos::NsProblemManager
{

public:

	enum Algorithm {HILL, ANNEALING};

	////////////////////////////////////// Configuration //////////////////////////////////////

	struct Configuration
	{
		virtual ~Configuration(void) {}

		virtual Algorithm algorithm (void) = 0;
		virtual std::ostream& configuration (std::ostream&) = 0;
		virtual std::ostream& statistics (std::ostream&) = 0;
	};

	////////////////////////////////////// HillConfiguration //////////////////////////////////////

	// Configuration for the Hill Climbing algorithm
	struct HillConfiguration : public Configuration
	{
		VariableHeuristic* 	variableHeuristic;
		ValueHeuristic* 	valueHeuristic;
		unsigned long 		maxStateRepeats;
		unsigned long 		maxAvoidAttempts;
		double	 		walkProb;
		unsigned long 		steps;
		unsigned long 		maxSteps;
		unsigned long 		restarts;

		HillConfiguration (VariableHeuristic* variableHeuristic_, ValueHeuristic* valueHeuristic_,
				unsigned long maxStateRepeats_, unsigned long maxAvoidAttempts_ = 5, double walkProb_ = 0.0) :
				variableHeuristic(variableHeuristic_), valueHeuristic(valueHeuristic_),
				maxStateRepeats(maxStateRepeats_), maxAvoidAttempts(maxAvoidAttempts_), walkProb(walkProb_) {}

		Algorithm algorithm (void) { return HILL; }
		std::ostream& configuration (std::ostream&);
		std::ostream& statistics (std::ostream&);
	};

	////////////////////////////////////// AnnealingConfiguration //////////////////////////////////////

	// Configuration for the Simulated Annealing algorithm
	struct AnnealingConfiguration : public Configuration
	{
		TemperatureScheduler* 	scheduler;
		unsigned long 		steps;
		unsigned long 		restarts;

		AnnealingConfiguration (TemperatureScheduler* scheduler_) : scheduler(scheduler_) {}

		Algorithm algorithm (void) { return ANNEALING; }
		std::ostream& configuration (std::ostream&);
		std::ostream& statistics (std::ostream&);
	};

protected:

	naxos::NsIntVarArray* 		varArray;
	// Record all previous solutions found so as to report only new ones
	ActiveWindow<std::string> previousSolutions;
	// Elapsed time for the most recent solution found
	double 				elapsedTime;

	// Configurations specific to the algorithm in use
	Algorithm 			usingAlgorithm;
	Configuration* 			conf;

	// For Tabu Search
	unsigned long 			tabuTenure;
	ActiveWindow<Assignment> 	tabuAssignments;
	// The best minConflicts found while searching for a solution;
	// Used for the aspiration criterion (improvement in the incumbent candidate solution)
	naxos::NsInt 			globalMinConflicts;

	unsigned long 			seed;


	void initialize (void);
	void reset (void);

	void solveHill (void);
	void solveAnnealing (void);

	std::string hashState(naxos::NsIndex, naxos::NsInt);

public:

	// Declared as public so that other classes can use the same instance to produce random numbers
	MTRand_int32 	random;


	LsProblemManager (unsigned long tabuTenure_ = 1, unsigned long seed_ = 1) : varArray(NULL), conf(NULL),
			tabuTenure(tabuTenure_), tabuAssignments(tabuTenure), seed(seed_), random(seed) { }
	virtual ~LsProblemManager (void) {}

	void label (naxos::NsIntVarArray& varArray_, Configuration* conf_);
	void nextSolution (void);

	bool tryAssignment (Assignment);
	void commitAssignment (Assignment);
	void revertToAssignment (Assignment);

	std::ostream& solutionToString (std::ostream&);
	std::ostream& configuration (std::ostream&);
	std::ostream& statistics (std::ostream&);

	void printTabu(void)
	{
		std::cerr << "\t\t\t\t\t\t\t\t\t\tTABU LIST: ";
		for (ActiveWindow<Assignment>::iterator it = tabuAssignments.begin() ; it != tabuAssignments.end() ; it++)
			std::cerr << "| " << it->first->lsIndex() << " - " << it->second << " ";
		std::cerr << std::endl;
	}
};





////////////////////////////////////// MaxConflictingVariable //////////////////////////////////////

// Returns variable participating in the most violated constraints
struct MaxConflictingVariable : public VariableHeuristic
{
	MaxConflictingVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		int 		maxViolations = 0;
		ItVector 	maxIts;
		ConfVariables 	conflictSet = pm.lsConflictingVars();
		for ( ConfVarsIterator it = conflictSet.begin() ; it != conflictSet.end() ; it++ )
		{
			int currentViolations = (*it)->lsViolatedConstraints().size();
			if ( currentViolations > maxViolations )
			{
				maxViolations = currentViolations;
				maxIts.clear();
			}
			if ( currentViolations == maxViolations ) maxIts.push_back(it);
		}

		// Tie break is random
		return *maxIts[ pm.random( maxIts.size() ) ];
	}
};


////////////////////////////////////// MinConflictingVariable //////////////////////////////////////

// Returns variable participating in the least violated constraints
struct MinConflictingVariable : public VariableHeuristic
{
	MinConflictingVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		ConfVariables 	conflictSet = pm.lsConflictingVars();
		int 		minViolations = (*conflictSet.begin())->lsViolatedConstraints().size();
		ItVector 	minIts;
		for ( ConfVarsIterator it = conflictSet.begin() ; it != conflictSet.end() ; it++ )
		{
			int currentViolations = (*it)->lsViolatedConstraints().size();
			if ( currentViolations < minViolations )
			{
				minViolations = currentViolations;
				minIts.clear();
			}
			if ( currentViolations == minViolations ) minIts.push_back(it);
		}

		// Tie break is random
		return *minIts[ pm.random( minIts.size() ) ];
	}
};


////////////////////////////////////// FirstVariable //////////////////////////////////////

// Returns the first variable
struct FirstVariable : public VariableHeuristic
{
	FirstVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		ConfVariables conflictSet = pm.lsConflictingVars();
		return *conflictSet.begin();
	}
};


////////////////////////////////////// BiggestDomainVariable //////////////////////////////////////

// Returns variable with the biggest domain
struct BiggestDomainVariable : public VariableHeuristic
{
	BiggestDomainVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		unsigned int 	biggestDomain = 0;
		ItVector 	biggestIts;
		ConfVariables 	conflictSet = pm.lsConflictingVars();
		for ( ConfVarsIterator it = conflictSet.begin() ; it != conflictSet.end() ; it++ )
		{
			unsigned int currentSize = (*it)->size();
			if ( currentSize > biggestDomain )
			{
				biggestDomain = currentSize;
				biggestIts.clear();
			}
			if ( currentSize == biggestDomain ) biggestIts.push_back(it);
		}

		// Tie break is random
		return *biggestIts[ pm.random( biggestIts.size() ) ];
	}
};


////////////////////////////////////// SmallestDomainVariable //////////////////////////////////////

// Returns variable with the smallest domain
struct SmallestDomainVariable : public VariableHeuristic
{
	SmallestDomainVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		ConfVariables 	conflictSet = pm.lsConflictingVars();
		unsigned int 	smallestDomain = (*conflictSet.begin())->size();
		ItVector 	smallestIts;
		for ( ConfVarsIterator it = conflictSet.begin() ; it != conflictSet.end() ; it++ )
		{
			unsigned int currentSize = (*it)->size();
			if ( currentSize < smallestDomain )
			{
				smallestDomain = currentSize;
				smallestIts.clear();
			}
			if ( currentSize == smallestDomain ) smallestIts.push_back(it);
		}

		// Tie break is random
		return *smallestIts[ pm.random( smallestIts.size() ) ];
	}
};


////////////////////////////////////// RandomVariable //////////////////////////////////////

// Returns a variable at random
struct RandomVariable : public VariableHeuristic
{
	RandomVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		ConfVariables 		conflictSet = pm.lsConflictingVars();
		int 			selection = pm.random( conflictSet.size() );
		ConfVarsIterator 	it;
		for ( it = conflictSet.begin() ; it != conflictSet.end() ; it++, selection-- )
			if ( selection == 0 ) break;

		return *it;
	}
};




////////////////////////////////////// MinConflictingValue //////////////////////////////////////

// Select value for selected variable that minimizes violated constraints
struct MinConflictingValue : public ValueHeuristic
{
	MinConflictingValue (LsProblemManager& pm_) : ValueHeuristic(pm_) { }

	naxos::NsInt select (naxos::NsIntVar& variable)
	{
		using namespace naxos;

		NsInt 	confValue = variable.lsValue();
		NsInt 	minConflicts = pm.lsViolatedConstraints().size();
		NsInt 	minConfValue = confValue;
		for ( NsInt currentValue = variable.min() ; currentValue <= variable.max() ; currentValue = variable.next(currentValue) )
		{
			// Skip assignment if it is the same as the current one
			if ( currentValue == confValue ) continue;

			// Try to assign the current value; it will fail if the assignment isn't allowed
			// On failure skip the current value and try the next one
			if ( !pm.tryAssignment( std::make_pair(&variable, currentValue) ) ) continue;

			NsInt currentConflicts = pm.lsViolatedConstraints().size();
			if ( currentConflicts < minConflicts )
			{
				// Update best assignment
				minConflicts = currentConflicts;
				minConfValue = currentValue;
			}
		}

		// Commit final assignment
		pm.commitAssignment( std::make_pair(&variable, minConfValue) );

		return minConfValue;
	}
};


////////////////////////////////////// RandomValue //////////////////////////////////////

// Select value at random
struct RandomValue : public ValueHeuristic
{
	RandomValue(LsProblemManager& pm_) : ValueHeuristic(pm_) {}

	naxos::NsInt select (naxos::NsIntVar& variable)
	{
		using namespace naxos;

		int 	selection = pm.random( variable.size() );
		NsInt 	confValue = variable.lsValue();
		NsInt 	selectedValue = 0;
		for ( NsInt currentValue = variable.min() ; currentValue <= variable.max() ; currentValue = variable.next(currentValue) )
		{
			selectedValue = currentValue;

			// Skip assignment if it is the same as the current one
			if ( currentValue == confValue ) continue;

			// Try to assign the current value; it will fail if the assignment isn't allowed
			// On failure skip the current value and try the next one
			if ( !pm.tryAssignment( std::make_pair(&variable, currentValue) ) ) continue;

			if ( selection-- == 0 ) break;

		}
		// Commit final assignment
		pm.commitAssignment( std::make_pair(&variable, selectedValue) );

		return selectedValue;
	}
};




////////////////////////////////////// BestImprovementVariable //////////////////////////////////////

// Returns the variable that causes the best improvement over the conflicting constraints
// NOTE: Can only be used with BestImprovementValue as a ValueHeuristic
struct BestImprovementVariable : public VariableHeuristic
{

private:

	naxos::NsInt bestValue;

public:

	BestImprovementVariable (LsProblemManager& pm_) : VariableHeuristic(pm_) {}

	VariablePtr select (void)
	{
		using namespace naxos;

		int 				minConflicts = pm.lsViolatedConstraints().size();
		ConfVariables 		conflictSet = pm.lsConflictingVars();
		ItVector 			bestIts;
		ValueVector 		bestValues;
		MinConflictingValue selectValue( pm );
		for ( ConfVarsIterator it = conflictSet.begin() ; it != conflictSet.end() ; it++ )
		{
			NsIntVar& variable = *(*it);
			NsInt 	currentValue = variable.lsValue();
			NsInt 	selectedValue = selectValue.select( variable );
			NsInt 	currentConflicts = pm.lsViolatedConstraints().size();

			if ( currentConflicts < minConflicts )
			{
				minConflicts = currentConflicts;
				bestIts.clear();
				bestValues.clear();
			}
			if ( currentConflicts == minConflicts )
			{
				bestIts.push_back( it );
				bestValues.push_back( selectedValue );
			}

			// Don't call `commitAssignment' instead; We don't want to update the tabu status
			variable.lsUnset();
			variable.lsSet( currentValue );
		}

		// Tie break is random
		unsigned int selection = pm.random( bestIts.size() );
		bestValue = bestValues[ selection ];
		return *bestIts[ selection ];
	}

	friend struct BestImprovementValue;
};

////////////////////////////////////// BestImprovementValue //////////////////////////////////////

struct BestImprovementValue : public ValueHeuristic
{

private:

	BestImprovementVariable& variableSelect;

public:

	BestImprovementValue(LsProblemManager& pm_, BestImprovementVariable& variableSelect_) : ValueHeuristic(pm_), variableSelect(variableSelect_) {}

	naxos::NsInt select (naxos::NsIntVar& variable)
	{
		// Commit final assignment
		pm.commitAssignment( std::make_pair(&variable, variableSelect.bestValue) );

		return variableSelect.bestValue;
	}
};




////////////////////////////////////// LogarithmicScheduler //////////////////////////////////////

struct LogarithmicScheduler : public TemperatureScheduler
{

private:

	unsigned int d;

public:

	LogarithmicScheduler(LsProblemManager& pm_, unsigned long stableSteps_, int d_) :
			TemperatureScheduler(pm_, stableSteps_), d(d_) {}

	double operator[] (unsigned long step)
	{
		return (100000 * d / log ( step ));
	}
	
	std::ostream& configuration (std::ostream&);
};

////////////////////////////////////// GeometricScheduler //////////////////////////////////////

struct GeometricScheduler : public TemperatureScheduler
{

private:

	double r;
	double previous;
	unsigned long previousStep;

public:

	GeometricScheduler(LsProblemManager& pm_, unsigned long stableSteps_, double r_) :
			TemperatureScheduler(pm_, stableSteps_), r(r_), previous(100000.0), previousStep(0) {}

	double operator[] (unsigned long step)
	{
		if ( step != previousStep )
		{
			previous = r * previous;
			previousStep = step;
		}
		return previous;
	}
	
	std::ostream& configuration (std::ostream&);
};




///////////////// Inline for speed, must therefore reside in header file /////////////////

inline void LsProblemManager::initialize (void)
{
	using namespace naxos;

	NsIntVarArray& 	variables = *varArray;

	for ( NsIndex i = 0, size = variables.size() ; i < size ; i++ )
	{
		NsInt 	currentValue, index, domainIndex = random(variables[i].size());

		for ( currentValue = variables[i].min(), index = 0 ; index < domainIndex ; index++, currentValue = variables[i].next(currentValue) ) ;

		//std::cerr << "TRYING TO LSSET " << i << std::endl;
		variables[i].lsSet(currentValue);
	}

	tabuAssignments.clear();
}


inline void LsProblemManager::reset (void)
{
	for ( naxos::NsIndex i = 0, size = varArray->size() ; i < size ; i++ ) (*varArray)[i].lsUnset();
}


inline bool LsProblemManager::tryAssignment (Assignment assignment)
{
	long currentConflicts = lsViolatedConstraints().size();
	// globalMinConflicts is -1 at the beginning of each search process;
	// Initialize or update its value here
	if ( globalMinConflicts == -1 || currentConflicts < globalMinConflicts ) globalMinConflicts = currentConflicts;

	// Temporary assign value to variable to compute the number of conflicting constraints
	assignment.first->lsUnset();
	assignment.first->lsSet( assignment.second );
	long nextConflicts = lsViolatedConstraints().size();

	// Allow assignment if it is not in the tabu list
	// OR if it is but it satisfies the aspiration criterion
	// (i.e. improves the incumbent candidate solution)
	if ( !tabuAssignments.find( assignment ) || nextConflicts < globalMinConflicts ) return true;

	//std::cerr << "\t\t\t\t\t\t\t\t\t\tTABU STATE IGNORED: " << assignment.first->lsIndex() << " - " << assignment.second << std::endl;
	return false;
}


inline void LsProblemManager::commitAssignment (Assignment assignment)
{
	assignment.first->lsUnset();
	assignment.first->lsSet( assignment.second );
	// Add assignment in the tabu set
	tabuAssignments.push( assignment );
}


inline void LsProblemManager::revertToAssignment (Assignment assignment)
{
	// Remove last assignment, from the tabu set
	tabuAssignments.pop_back();
	// Revert to assignment
	commitAssignment(assignment);
}


inline std::string LsProblemManager::hashState(naxos::NsIndex variable, naxos::NsInt value)
{
	std::ostringstream out;
	for ( naxos::NsIndex i = 0, size = varArray->size() ; i < size ; i++ ) out << (*varArray)[i].lsValue() << "|";
	out << "|" << variable << "->" << value;

	return MD5_string( out.str() );
}



} // end namespace


#endif // LOCAL_S_H

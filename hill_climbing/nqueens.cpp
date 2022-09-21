#include "../../naxos/naxos.h"
#include "../localS.h"

#include <iostream>
#include <ctime>


using namespace std;
using namespace naxos;
using namespace localS;


int main (int argc, char *argv[])
{
	try {

		if ( argc == 1 ) { cerr << "USAGE: N stateRepeats avoidAttempts walkProb tabuTenure seed" << endl; exit(1); }

		int N = (argc > 1) ? atoi(argv[1]) : 8;
		unsigned long stateRepeats = (argc > 2) ? atol(argv[2]) : 5;
		unsigned long avoidAttempts = (argc > 3) ? atol(argv[3]) : 2;
		double walkProb = (argc > 4) ? atof(argv[4]) : 0;
		unsigned long tabuTenure = (argc > 5) ? atol(argv[5]) : 2;
		unsigned long seed = (argc > 6) ? atol(argv[6]) : time(NULL);

		LsProblemManager  pm( tabuTenure, seed );

		// HILL CLIMBING //
		MaxConflictingVariable 		selectVariable( pm );
		//MinConflictingVariable 	selectVariable( pm );
		//FirstVariable 		selectVariable( pm );
		//RandomVariable 		selectVariable( pm );
		//BiggestDomainVariable 	selectVariable( pm );
		//SmallestDomainVariable 	selectVariable( pm );
		MinConflictingValue 		selectValue( pm );
		//RandomValue 			selectValue( pm );
		// NOTE: Must be used together //
		//BestImprovementVariable 	selectVariable( pm );
		//BestImprovementValue 		selectValue( pm, selectVariable );
		LsProblemManager::HillConfiguration conf( &selectVariable, &selectValue, stateRepeats, avoidAttempts, walkProb );

		// SIMULATED ANNEALING //
		//LogarithmicScheduler scheduler( pm, 3, 56 );
		//GeometricScheduler scheduler( pm, 3, 0.9991 );
		//LsProblemManager::AnnealingConfiguration conf( &scheduler );

		// PROBLEM STATEMENT //
		NsIntVarArray  Var, VarPlus, VarMinus;
		for (int i=0;  i < N;  ++i)
		{
			Var.push_back( NsIntVar(pm, 0, N-1) );
			VarPlus.push_back(  Var[i] + i );
			VarMinus.push_back( Var[i] - i );
		}
		pm.add( NsAllDiff(Var) );
		pm.add( NsAllDiff(VarPlus) );
		pm.add( NsAllDiff(VarMinus) );

		// LABELING //
		pm.label(Var, &conf);

		// SOLVING //
		//cout << N << "\t";
		pm.configuration( cout );
		//while (true)
		{
			pm.nextSolution();
			pm.solutionToString( cout );
			pm.statistics( cout );
		}

	} catch (exception& exc) {
		cerr << exc.what() << "\n";
	} catch (...) {
		cerr << "Unknown exception" << "\n";
	}
}

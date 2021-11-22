// ***************************************
// Worm Chemotaxis
// ***************************************

#include "CTRNN.h"
#include "WormAgent.h"
#include "TSearch.h"
#include <sstream>


#define PRINTTOFILE



// Global constants
const int		CircuitSize				= 2;
const double	RunDuration				= 500.0;
const int		MaxRepetitions			= 5;
const int		MaxReplacements			= 10;
const double	WeightRange				= 15.0;
const double	SensoryWeightRange		= 1500.0;
const double	BiasRange				= 15.0;
const double	CloseEnoughRadius		= 0.5;
const double	MinSensorN				= 10*StepSize;
const double	MaxSensorN				= HST;
const double	MinSensorM				= 10*StepSize;
const double	MaxSensorM				= HST;
const double	MinSensorD				= 10*StepSize;
const double	MaxSensorD				= HST;
const double	MinNeckTurnGain			= 1.0;
const double	MaxNeckTurnGain			= 3.0;
//const double	PirouetteProb			= 0.000333333; //(2/60)*0.01; // They reverse about twice a minute (according to Shawn and Serge). In 500 secs, that's 16.66 on avg.
const double	PirouetteProb			= 0.00004; // Smaller. So that it doesn't evolve to depend on it. On avg, 2 per trial (500secs) [(2/500) * 0.01]
const double	SizeOfStep				= 0.002;	// Size of steps for the riverchip experiment

const double	RiverChipSteepness		= 50.0;

// Global variables
int	VectSize = 9;

// ------------------------------------
// Genotype-Phenotype Mapping Functions
// ------------------------------------
void GenPhenMapping(TVector<double> &gen, TVector<double> &phen)
{
	phen(1) = MapSearchParameter( gen(1), -SensoryWeightRange, SensoryWeightRange); // w_on
	phen(2) = MapSearchParameter( gen(2), -SensoryWeightRange, SensoryWeightRange);	// w_off
	phen(3) = MapSearchParameter( gen(3), 0.0, WeightRange);						// w_osc
	phen(4) = MapSearchParameter( gen(4), -WeightRange, WeightRange);				// w_osc
	phen(5) = MapSearchParameter( gen(5), -BiasRange, BiasRange);					// bias or threshold
	phen(6) = MapSearchParameter( gen(6), MinSensorN, MaxSensorN);					// Sensory cell integration time, current
	phen(7) = MapSearchParameter( gen(7),  MinSensorM, MaxSensorM);					// Sensory cell integration time, past
	phen(8) = MapSearchParameter( gen(8),  MinSensorD, MaxSensorD);					// Sensory cell time constant delay
	phen(9) = MapSearchParameter( gen(9), MinNeckTurnGain, MaxNeckTurnGain);		// Neck turning gain
}

// ------------------------------------
// Fitness function
// ------------------------------------
double EvaluationFunction(TVector<double> &v, RandomState &rs)
{
	double t,fitness=0.0;
	double fA,accdist,totaldist;
	int k,condition,repetitions,replacements;
	TVector<double> phenotype;

	phenotype.SetBounds(1, VectSize);
	GenPhenMapping(v, phenotype);
	WormAgent Worm(CircuitSize);
	Worm.SetParameters(phenotype);
	Worm.InitialiseAgent(StepSize);

	k=0;
	for (condition = 4; condition <= 4; condition++){
		for (repetitions = 1; repetitions <= MaxRepetitions; repetitions++)
		{
			Worm.ResetAgentIntState(rs);
			for (replacements = 1; replacements <= MaxReplacements; replacements++)
			{
				Worm.ResetAgentsBody(rs);
				Worm.ResetChemCon(rs);
				Worm.UpdateChemCon(rs);
				Worm.InitialiseSensorHistory();
				accdist = 0.0;
				for (t = StepSize; t <= RunDuration; t += StepSize)
				{
					Worm.Step(StepSize,rs,t);
					Worm.UpdateChemCon(rs);
					Worm.UpdateChemConHistory();
					Worm.UpdateSensors();

					if (rs.UniformRandom(0.0, 1.0) < PirouetteProb)
						Worm.SetOrientation(rs.UniformRandom(0, 2*Pi));

					if (condition == 1)
						Worm.SetOnCell(0.0);
					if (condition == 2)
						Worm.SetOffCell(0.0);

					accdist += Worm.DistanceToCentre();
				}

				totaldist = (accdist/(RunDuration/StepSize));
				//fA = (MaxDist - totaldist)/MaxDist;
				fA = totaldist;
				fA = fA < 0 ? 0.0 : fA;
				fitness += fA;
			}
		}
		k++;
	}
	return fitness/(k*MaxRepetitions*MaxReplacements); // MaxReplacements			= 10; const int		MaxRepetitions			= 5; k=1
}


// ------------------------------------
// Display functions
// ------------------------------------
void EvolutionaryRunDisplay(int Generation, double BestPerf, double AvgPerf, double PerfVar)
{
	cout << Generation << " " << BestPerf << " " << AvgPerf << " " << PerfVar << endl;
}

void ResultsDisplay(TSearch &s)
{
	double p;
	TVector<double> bestVector;
	ofstream BestIndividualFile;
	TVector<double> phenotype;
	phenotype.SetBounds(1,VectSize);
	// Save the genotype of the best individual
	bestVector = s.BestIndividual();
	BestIndividualFile.open("best.gen.dat");
	BestIndividualFile << bestVector << endl;
	BestIndividualFile.close();
	// Also show the best individual in the Circuit Model form
	BestIndividualFile.open("best.ns.dat");
	GenPhenMapping(bestVector, phenotype);
	WormAgent Worm(CircuitSize);
	Worm.SetParameters(phenotype);
	Worm.InitialiseAgent(StepSize);
	BestIndividualFile << Worm.NervousSystem;
	BestIndividualFile << endl;
	for (int i = 1; i <= CircuitSize; i++){
		p = Worm.onWeight(i);
		BestIndividualFile << p << " ";
	}
	BestIndividualFile << endl;
	for (int i = 1; i <= CircuitSize; i++){
		p = Worm.offWeight(i);
		BestIndividualFile << p << " ";
	}
	BestIndividualFile << endl;
	for (int i = 1; i <= CircuitSize; i++){
		p = Worm.cpgWeight(i);
		BestIndividualFile << p << " ";
	}
	BestIndividualFile << endl;
	BestIndividualFile << Worm.SensorN() << " " << endl;
	BestIndividualFile << Worm.SensorM() << " " << endl;
	BestIndividualFile << Worm.SensorD() << " " << endl;
	BestIndividualFile << Worm.OutputGain() << " " << endl;
	BestIndividualFile.close();
	BestIndividualFile.open("quickgen.dat");
	BestIndividualFile << Worm.onWeight(1) << " " << Worm.offWeight(1) << " " << Worm.cpgWeight(1) << " " << Worm.NervousSystem.ConnectionWeight(1,1) << " " << Worm.NervousSystem.NeuronBias(1) << " " << Worm.SensorN() << " " << Worm.SensorM() << " " << Worm.SensorD() << " " << Worm.OutputGain() << endl;
	BestIndividualFile.close();

	//BehavioralOverview();
}

// ------------------------------------
// The main program
// ------------------------------------
//#ifdef EVOLVE
int main (int argc, const char* argv[])
{
	long IDUM=-time(0);
	TSearch s(VectSize);
	TVector<double> phenotype;
	phenotype.SetBounds(1,VectSize);

	// redirect standard output to a file
#ifdef PRINTTOFILE
	ofstream file;
	file.open ("evol.dat");
	cout.rdbuf(file.rdbuf());
#endif

	// Configure the search
	s.SetRandomSeed(IDUM);
	s.SetPopulationStatisticsDisplayFunction(EvolutionaryRunDisplay);
	s.SetSearchResultsDisplayFunction(ResultsDisplay);
	s.SetSelectionMode(RANK_BASED);			//{FITNESS_PROPORTIONATE,RANK_BASED}
	s.SetReproductionMode(HILL_CLIMBING);	// {HILL_CLIMBING, GENETIC_ALGORITHM}
	s.SetPopulationSize(10);
	s.SetMaxGenerations(100);
	s.SetMutationVariance(0.05);
	s.SetCrossoverProbability(0.5);
	s.SetCrossoverMode(TWO_POINT);			//{UNIFORM, TWO_POINT}
	s.SetMaxExpectedOffspring(1.1);
	s.SetElitistFraction(0.0);				//Default is 0.0.
	s.SetSearchConstraint(1);
	s.SetCheckpointInterval(0);
	s.SetReEvaluationFlag(1);				// CRUCIAL

	s.SetEvaluationFunction(EvaluationFunction);
	s.ExecuteSearch();

	return 0;
}


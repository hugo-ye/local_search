#ifndef AUXILIARY_H
#define AUXILIARY_H

#include <cmath>
#include <ctime>

namespace localS
{


template <class TemplType>
class ActiveWindow : public std::deque<TemplType>
{

private:

	unsigned int width_;

public:

	// If width == 0 behave like a normal Deque
	// Otherwise keep the last `width' elements
	ActiveWindow (unsigned int width__ = 0) : width_(width__) {}

	unsigned int width () { return width_; }
	void width (unsigned int width__) { width_ = width__; }

	void push (TemplType item)
	{
		this->push_back(item);
		if ( width_ != 0 && this->size() > width_ ) this->pop_front();
	}

	unsigned int search (TemplType item)
	{
		int occurrences = 0;
		for (typename std::deque<TemplType>::iterator it = this->begin() ; it != this->end() ; it++)
			if ( *it == item ) occurrences++;

		return occurrences;
	}

	bool find (TemplType item)
	{
		for (typename std::deque<TemplType>::iterator it = this->begin() ; it != this->end() ; it++)
			if ( *it == item ) return true;

		return false;
	}
};



const double math_e = 2.7182818284590452354;

inline double exponentialDecay(double t, double N0 = 1, double l = -1)
{
	return N0 * pow(math_e, -l * t);
}


inline std::string MD5_string (std::string str)
{
	MD5 context;

	context.update( (unsigned char*) str.c_str(), str.size() );
	context.finalize();

	return context.hex_digest();
}



class Timer
{

private:

	enum State {INACTIVE, ACTIVE, PAUSED};

	char 	state_;
	double 	elapsed_;
	clock_t start_;

public:
	
	Timer () : state_(INACTIVE), elapsed_(0.0) { }

	void start ()
	{
		start_ = clock();
		state_ = ACTIVE;
		elapsed_ = 0.0;
	}

	void pause ()
	{
		if ( state_ == ACTIVE )
		{
			elapsed_ += elapsed();
			state_ = PAUSED;
		}
	}
	void unpause ()
	{
		if ( state_ == PAUSED )
		{
			start_ = clock();
			state_ = ACTIVE;
		}
	}

	double elapsed ()
	{
		if ( state_ == ACTIVE )
			return elapsed_ + ( ((double) (clock() - start_)) / CLOCKS_PER_SEC );
		else if ( state_ == PAUSED )
			return elapsed_;
		else
			return 0.0;
	}
};



} // end namespace


#endif // AUXILIARY_H

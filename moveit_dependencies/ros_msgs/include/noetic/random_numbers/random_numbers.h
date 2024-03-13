/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef RANDOM_NUMBERS_RANDOM_NUMBERS_
#define RANDOM_NUMBERS_RANDOM_NUMBERS_

#if 1

#include <stdint.h>
#include <random>

namespace random_numbers
{
/** \brief Random number generation (wrapper for boost). An instance of this class
    cannot be used by multiple threads at once (member functions
    are not const). However, the constructor is thread safe and
    different instances can be used safely in any number of
    threads. It is also guaranteed that all created instances will
    have a "random" random seed. */
class RandomNumberGenerator
{
public:

  /** \brief Constructor. Always sets a "random" random seed */
  RandomNumberGenerator(void) {}

  /** \brief Constructor. Allow a seed to be specified for deterministic behaviour */
  RandomNumberGenerator(uint32_t seed) : generator_(seed) { first_seed_ = seed; }

  /** \brief Generate a random real between 0 and 1 */
  double uniform01(void)
  {
	  return uni_(generator_);
  }

  /**
   * @brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound)
   * @param lower_bound The lower bound
   * @param upper_bound The upper bound
   */
  double uniformReal(double lower_bound, double upper_bound)
  {
    return (upper_bound - lower_bound) * uni_(generator_) + lower_bound;
  }

  /** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
  double gaussian01(void)
  {
    return normal_(generator_);
  }

  /** \brief Generate a random real using a normal distribution with given mean and variance */
  double gaussian(double mean, double stddev)
  {
    return normal_(generator_) * stddev + mean;
  }

  /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w)
   * @param value[4] A four dimensional array in which the computed quaternion will be returned
   */
	void quaternion(double value[4])
	{
		double x0 = uni_(generator_);
		double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
		double t1 = 2.0 * M_PI * uni_(generator_),
			t2 = 2.0 * M_PI * uni_(generator_);
		double c1 = cos(t1), s1 = sin(t1);
		double c2 = cos(t2), s2 = sin(t2);
		value[0] = s1 * r1;
		value[1] = c1 * r1;
		value[2] = s2 * r2;
		value[3] = c2 * r2;
	}

  /** \brief Generate an integer uniformly at random within a specified range (inclusive) */
  int uniformInteger(int min, int max)
  {
      std::uniform_int_distribution dis(min, max);
	  return dis(generator_);
  }

  /**
   * \brief Allow the randomly generated seed to be saved so that experiments / benchmarks can be recreated in the future
   */
  uint32_t getFirstSeed() { return first_seed_; }

private:
	uint32_t first_seed_;
	std::default_random_engine generator_;
	std::uniform_real_distribution<double> uni_;
	std::normal_distribution<double> normal_;
};

}

#else
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

namespace random_numbers
{
/** \brief Random number generation (wrapper for boost). An instance of this class
    cannot be used by multiple threads at once (member functions
    are not const). However, the constructor is thread safe and
    different instances can be used safely in any number of
    threads. It is also guaranteed that all created instances will
    have a "random" random seed. */
class RandomNumberGenerator
{
public:
  
  /** \brief Constructor. Always sets a "random" random seed */
  RandomNumberGenerator(void);

  /** \brief Constructor. Allow a seed to be specified for deterministic behaviour */
  RandomNumberGenerator(boost::uint32_t seed);
  
  /** \brief Generate a random real between 0 and 1 */
  double uniform01(void)
  {
    return uni_();
  }
  
  /**
   * @brief Generate a random real within given bounds: [\e lower_bound, \e upper_bound)
   * @param lower_bound The lower bound
   * @param upper_bound The upper bound
   */
  double uniformReal(double lower_bound, double upper_bound)
  {
    return (upper_bound - lower_bound) * uni_() + lower_bound;
  }

  /** \brief Generate a random real using a normal distribution with mean 0 and variance 1 */
  double gaussian01(void)
  {
    return normal_();
  }
  
  /** \brief Generate a random real using a normal distribution with given mean and variance */
  double gaussian(double mean, double stddev)
  {
    return normal_() * stddev + mean;
  }
  
  /** \brief Uniform random unit quaternion sampling. The computed value has the order (x,y,z,w)
   * @param value[4] A four dimensional array in which the computed quaternion will be returned
   */
  void quaternion(double value[4]);
  
  /** \brief Generate an integer uniformly at random within a specified range (inclusive) */
  int uniformInteger(int min, int max)
  {
    boost::uniform_int<> dis(min, max);  
    return dis(generator_);
  }

  /**
   * \brief Allow the randomly generated seed to be saved so that experiments / benchmarks can be recreated in the future
   */
  boost::uint32_t getFirstSeed();
  
private:
  
  boost::mt19937                                                           generator_;
  boost::uniform_real<>                                                    uniDist_; 
  boost::normal_distribution<>                                             normalDist_;
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> >        uni_;  
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normal_;
};

}

#endif

#endif

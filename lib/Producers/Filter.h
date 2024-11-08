/*
 * Copyright 2015 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FILTER_H
#define FILTER_H

#include <vector>
#include <deque>
#include <cmath>
#include <iostream>
#include <cstdlib>

namespace FilterKin
{
	template<typename T> class Filter
	{
		public:
			Filter ( const std::vector<T>& aCoeff, const std::vector<T>& bCoeff );
			Filter ( const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, std::vector<T> pastData );
			Filter(){};

			~Filter();

			Filter ( const Filter& other )
			{
				_aCoeff = other._aCoeff;
				_bCoeff = other._bCoeff;
			}


			Filter& operator= ( const Filter& other )
			{
				_aCoeff = other._aCoeff;
				_bCoeff = other._bCoeff;
				_pastData = other._pastData;
				_pastDataFilter = other._pastDataFilter;
				return *this;
			}

			bool operator== ( const Filter& other )
			{
				return _aCoeff == other._aCoeff && _bCoeff == other._bCoeff && _pastData == other._pastData && _pastDataFilter == other._pastDataFilter;
			}

			void init(const std::vector<T>& aCoeff, const std::vector<T>& bCoeff, std::vector<T> pastData);

			T filter ( const T& data );

			void reset();	// Clear filter to restart calculations

		protected:

			typedef typename std::vector<T>::iterator vectTI;
			typedef typename std::vector<T>::const_iterator vectTCI;

			std::vector<T> _aCoeff; //!< A coefficients.
			std::vector<T> _bCoeff; //!< B coefficients.
			std::deque<T> _pastData; //!< Past data.
			std::deque<T> _pastDataFilter; //!< Past data.
	};
	
	template<typename T> class AvrFilt
	{
		public:
			AvrFilt ( int size );
			AvrFilt ( std::vector<T> pastData, int size );

			~AvrFilt();

			T filter ( const T& data );

		protected:

// 			typedef typename std::deque<T>::iterator DeqTCI;
			typedef typename std::deque<T>::const_iterator DeqTCI;

			std::deque<T> _pastData; //!< Past data.
			int _size;
	};
}

#include "Filter.cpp"

#endif // FILTER_H

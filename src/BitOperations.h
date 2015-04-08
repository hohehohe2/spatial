#ifndef hohe_BitOperations_H
#define hohe_BitOperations_H

//#include <intrin.h>
//#include <emmintrin.h>

namespace hohehohe2
{

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//! Collection of bit operation functions.
struct BitOperations
{

	//! Count the number of successive 0 bits from MSB.
	static unsigned int countLeadingZeros32(unsigned int x)
	{
		if (x ==0)
		{
			return 32;
		}

		unsigned int n = 0;
		if (x <= 0x0000ffff) {n += 16; x <<= 16;}
		if (x <= 0x00ffffff) {n +=  8; x <<= 8; }
		if (x <= 0x0fffffff) {n +=  4; x <<= 4; }
		if (x <= 0x3fffffff) {n +=  2; x <<= 2; }
		if (x <= 0x7fffffff) {++n;}
		return n;
	}

	//! Calculate morton code.
	static unsigned int calcMortonCode32(unsigned int x, unsigned int y, unsigned int z)
	{
		//Since 3d objects are pulled by gravity and piled on the ground, we can expect that
		//there are many objects that shares the same y value. The code is expecting the result
		//morton code as continuous as possible by placing the bits in y to the lower bits so that
		//the hash (in a spatial hashing) diverges.
		return
			insertBitsForMorton32_(y) |
			insertBitsForMorton32_(x) << 1 |
			insertBitsForMorton32_(z) << 2;
	}

	//! Calculate morton code.
	static unsigned int calcMortonCode32(unsigned int x, unsigned int y)
	{
		return
			insertBitsForMorton32_(y) |
			insertBitsForMorton32_(x) << 1;
	}

	//To be implemented.
	//static __m128i calcMortonCode32x4(__m128i x, __m128i y)
	//static __m128i calcMortonCode32x4(__m128i x, __m128i y, __m128i z)

private:

	static unsigned int insertBitsForMorton32_(unsigned int x)
	{
		x = (x | (x << 16)) & 0x030000FF;
		x = (x | (x <<  8)) & 0x0300F00F;
		x = (x | (x <<  4)) & 0x030C30C3;
		x = (x | (x <<  2)) & 0x09249249;
		return x;
	}
};

}

#endif

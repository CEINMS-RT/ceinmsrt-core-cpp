	#ifndef	PARAMETERS_H													
	#define	PARAMETERS_H													
															
	//UNCHECK	NEXT	LINE	FOR	PARALLEL	VERSION	OF	THE	PROGRAM						
//	#include	singleORparallel.h													
															
	#pragma	warning(disable:	4267)												
	#pragma	warning(disable:	4018)												
	#pragma	warning(disable:	4101)//unreferenced	local	variable										
	//	parameters.h													
	//#define	TNTlib	//if	tnt	doesn't	link	disable	this	line						
	#define	MAIN//	for	multi-scale	purposes										
	//#define	MAIN_PDB	//	for	analysing	PDB	files								
	//#define	TEST_RANDOM	//for	testing	random	generator;	contains	main	inside						
	//#define	MAIN_ENE													
	//#define	MAIN_CHARMM	//attempted	to	be	most	complete	fine	level	simulation					
	//#define	MAIN_SMMP													
	//#define	MAIN_CHUNKS													
															
															
	#define	USE_TIMING													
	#define	SQR(x)	(x)*(x)												
															
	#include	<string>													
	#include	<vector>//disable	this	line	and	enable	next	one	for	gcc2.9	and	older			
															
	#include	<iostream>													
	#include	<fstream>													
	#include	<cassert>													
	#include	<cmath>													
	#include<sstream>//needed	just	for	converting	int	to	string	in	residues.cpp						
	#include<iomanip>//used	only	in	atom.cpp	and	residues.cpp									
	#include<ctime>//used	for	timing	process											
	//#include<algorithm>//used	for	vector	copy											
	#include	<functional>//for	sorting												
	#include	<algorithm>													
															
	const	std::string	DIRECTORY("prot1/");												
	const	int	LASTSAVEATOM=-1;	//250	to	cut	after	prolint	in	all20;//-1;//minus	one	to	save	whole	protein
															
	const	int	MAX_LEVELS=24;//number	of	letters	in	alphabet								
	const	int	NUM_TOR_CLASSES=19;//from	smmp											
	const	int	NUM_ATOM_TYPES=18;//from	smmp											
	const	int	NUM_AA=30;//20;//after	including	water	it	became	number	of	different	residues				
	//since	TIP3	is	treated	as	residue	as	well.							
	const	int	MAX_CHI=5;//max	of	five	chi-angles	can	be	in	a	side	chain			
	const	int	MAX_BONDS=4;//	maximum	of	bonds	an	atom	can	have					
	const	int	PEPTIDE_ATOM_TYPES_FULL=35;												
	const	int	DOUBLE_IO_PRECISION=7;//number	of	decimal	figures	in	saving/reading	double						
	const	double	MAX_DOUBLE=1e308;												
	const	double	TINY_DOUBLE=1e-120;												
	const	double	CALCULATION_ERROR=1e-15;												
	//from	msdn	maximum	double	=	+/-1.79769313486232e308									
	namespace	chem_mass	{												
	const	double	H=1.008;												
	const	double	C=12.011;												
	const	double	N=14.007;												
	const	double	O=15.999;												
	const	double	S=32.060;												
	}														
															
	#ifndef	M_PI													
	#define	M_PI	3.141592654												
	#define	M_PI_2	1.570796327												
	#endif														
															
	namespace	table{													
	const	double	PI	=	3.14159265358979323846;										
	const	double	TWOPI	=	6.28318530717958647692;//2pi										
	const	double	DEG_RAD	=	57.29577951308232087679815;//57.29										
	const	double	RAD_DEG	=	0.017453292519943295769236;//0.017										
	const	double	ONE	=	1.0;										
	const	double	ZERO	=	0.0;										
	const	double	SQRT2	=	1.41421356237309504880;										
	const	double	ELE_COEFF	=	332.0716;//adjusted,	not	verified!!!								
	const	double	temp_coeff=503.94;//Hansmann	uses	503.08644										
	const	double	kcal=4190.;//Joules												
	const	double	inv_k=0.120272;//	1/k_b,	K*mol/J										
	/*i	use	inverse	boltzmann	constant.	in	exponents	you	usually	have	exp(-inv_k*E/T),				
	if	energy	is	in	kcal/mol	then	inv_k	must	be	in	K*mol/kcal.				
	inv_k=1/k_b=1/(1.38065E-23	J/K)=1K/(1.38065*6.022142	J/mol)												
	inv_k=0.120272	K*mol/J													
	(from	physics.nist.gov)													
															
	but	then	there	are	at	least	three	definitions	of	1calorie					
	1	cal=4.184J													
	1	cal=4.186J													
	1	cal=4.190J(i	used	this)											
	1	cal=4.1868J	(from	Ooi	et	al.,	Proc.Natl.Acad.Sci.,	v.84,	pp.3086-3090,	may	1987)				
	so														
	inv_k=503.22	K*mol/kcal													
	inv_k=503.46	K*mol/kcal													
	inv_k=503.94	K*mol/kcal(i	use	this)											
	this	is	inverse	boltzmann	constant										
															
	i	have	in	my	program										
	const	double	temp_coeff=503.94;												
															
	typical	value	at	T=300K	of	beta									
	beta=inv_k/300K=1.68	mol/kcal													
	*/														
															
	}														
	//GLOBAL	FUNCTIONS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!													
	inline	int	getIntTemperature(int	i)											
	{														
	double	t;													
	t=250+50*(i);														
	//	t=250+i*(131+i*(-28.4+2.3*i));													
	int	intt=(int)t;													
	return	intt;													
	}														
															
	inline	std::string	itos(int	i)	//	convert	int	to	string						
	{														
	std::stringstream	s;													
	s	<<	i;												
	return	s.str();													
	}														
	inline	double	elapse(int	id,time_t	st,time_t	fn)									
	{//usage:	#include<ctime>	time_t	start,finish;	time(&start);	...									
	//usage:	time(&finish);	elapse(start,finish);												
	double	df=difftime(fn,st);													
	int	dmin=0;													
	if(df>=60)	{													
	dmin=(int)(df/60);														
	df-=dmin*60;														
	}														
	//	cout<<"Time	span	min:sec	<<dmin<<':'<<df<<\n";										
	std::cerr<<"(single	proc)	Time	span	min:sec	<<id<<' '<<dmin<<':'<<df<<\n";									
	return	df;													
	}														
															
															
	#endif														
															

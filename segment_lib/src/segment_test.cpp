#include <segment_lib/segment_lib.h>
#define makeseg  segment_lib::makePathSegment
#define printseg segment_lib::printPathSegment

int main(int argc, char** argv)
{
  const double pi = 3.1415926;
	
	precision_navigation_msgs::PathSegment p;


/*
	printf("(-1,-1,3/4p) -> (-1,1,1/4p)\n");
	p = makeseg(-1,-1,3*pi/4,
	            -1, 1,pi/4);
	printseg(p);
	
	printf("(-1,1,-3/4p) -> (-1,-1,-1/4p)\n");
	p = makeseg(-1, 1,5*pi/4,
	            -1,-1,-pi/4);
	printseg(p);
	
	printf("(1,-1,1/4p) -> (1,1,3/4p)\n");
	p = makeseg(1,-1,pi/4,
	            1, 1,3*pi/4);
	printseg(p);
	
	printf("(1,-1,1/4p) -> (1,1,4/4p) (bad)\n");
	p = makeseg(1,-1,pi/4,
	            1, 1,pi);
	printseg(p);
	
	printf("(1,0,1/2p) -> (-1,0,-1/2p)\n");
	p = makeseg( 1.0/sqrt(2.0),1.0/sqrt(2.0),3*pi/4,
	            -1,0,-pi/2);
	printseg(p);
*/

	printf("(1,1,1/4p) -> (1,1,1/4p) (degenerate)\n");
	p = makeseg( 1,1,1*pi/4,
	             1,1,1*pi/4);

	printf("(1,1,1/4p) -> (6,6,1/4p) (bad angle)\n");
	p = makeseg( 1,1,1*pi/4+.1,
	             6,6,1*pi/4+.1);
	printseg(p);
	
	printf("(1,1,1/4p) -> (1,1,pi)\n");
	p = makeseg( 1,1,pi/4,
	             1,1,pi);
	printseg(p);
	
	printf("(1,1,.25p) -> (1,1,-.9pi)\n");
	p = makeseg( 1,1, .25*pi,
	             1,1,-.90*pi);
	printseg(p);
	
	return(0);
}

#include "NewVect.h"
#include <string.h>
//#include "MyMath.h"
#include "My_Math.h"
/**
 * @function    : 
 * @Description : ������������һ������
**/
void set_vect(vect_t *p_vect, point_t *p_start, point_t *p_end)
{
	memcpy(&p_vect->s, p_start, sizeof(point_t));
	memcpy(&p_vect->e, p_end,   sizeof(point_t));
}

void SetPoint(point_t * p_point, int32_t x, int32_t y, float ang)
{
    p_point->x = x;
    p_point->y = y;
    p_point->ang = ang;
}
/**
 * @function    : 
 * @Description : ���������һ�㵽�����Ĵ��ߵĴ���
 * @param       : ������ָ�룬�������ָ�룬�����ָ��
**/
void get_foot_point(vect_t *p_vect,point_t *p0, point_t * foot_point)
{
	int64_t x0 = p0->x;
	int64_t y0 = p0->y;					/*������*/
	int64_t x1 = p_vect->s.x;
	int64_t y1 = p_vect->s.y;		  	/*�����������*/
	int64_t x2 = p_vect->e.x;				  
	int64_t y2 = p_vect->e.y;			/*�����յ�����*/
	if (x1 == x2)						/*��Ŀ��ֱ�ߴ�ֱ��X��*/
	{
		foot_point->x = x1;
		foot_point->y = y0;
	} 
	else if (y1 == y2)					/*��Ŀ��ֱ�ߴ�ֱ��Y��*/
	{
		foot_point->x = x0;
		foot_point->y = y1;  				
	} 
	else								/*һ�����*/
	{
		foot_point->x = p0->x - (float)((y1 - y2)*(x0*y1 - x1*y0 - x0*y2 + x2*y0 + x1*y2 - x2*y1))
		                        /(square(x1) - 2*x1*x2 + square(x2) + square(y1) - 2*y1*y2 + square(y2));
		foot_point->y = (float)(x1-x2) * (foot_point->x-x0) / (y2-y1) + y0; 
	}
}

/*
 *	p2v_vertical_dist:	��㵽�����Ĵ�ֱ����
 *	p_vect:				����
 *	p0:					������
 *	����ֵ��			p0�������Ĵ�ֱ���루��������Ϊ��Ե�Y�ᣬ��p0�����X����Ϊ�����򷵻ظ�ֵ��
 */
int32_t p2v_vertical_dist(vect_t *p_vect, point_t *p0)
{
	point_t f_point;    /*footpoint*/
	vect_t  ver_vect;	/*��������*/

	get_foot_point(p_vect, p0, &f_point);	/*���߽��㼰���λ��*/
	set_vect(&ver_vect, &f_point, p0);	    /*���Լ����߽��㹹�ɵ�����*/
							
	return  get_vect_len(&ver_vect);			/*����*/
}

/*
 *	p2v_parallel_dist:	��ȡ�㵽���������ƽ�������������ϵľ���
 *	p_vect:				����ָ��
 *	p0��				������
 *	����ֵ��			p0�����������ƽ�������������ϵľ���
 */
int32_t p2v_parallel_dist(vect_t *p_vect, point_t *p0)
{
	point_t f_point;			/*���߽���*/
	vect_t  par_vect;	   	/*�������ƽ������������������*/
				
	get_foot_point(p_vect, p0, &f_point);		/*���߽���*/
	set_vect(&par_vect, &p_vect->s, &f_point);	/*ƽ������*/
	//�ж����������ǲ���ͬ������Ƿ�����ô���صľ���ֵ�Ǹ���
    if ((p_vect->e.x - p_vect->s.x) * (par_vect.e.x - par_vect.s.x) + 
        (p_vect->e.y - p_vect->s.y) * (par_vect.e.x - par_vect.s.y) < 0)
        return -get_vect_len(&par_vect);				/*����*/	
    else 
        return get_vect_len(&par_vect);
}


uint32_t get_vect_len(vect_t *p_vect)
{
	int64_t x, y, l;
	x = p_vect->e.x - p_vect->s.x;
	y = p_vect->e.y - p_vect->s.y;
	l = my_sqrt(square(x)+square(y));
	return l;
}
uint32_t get_point_dis(point_t * p1, point_t * p2)
{
    vect_t vect;
    memcpy(&vect.s,p1,sizeof(point_t));
    memcpy(&vect.e,p2,sizeof(point_t));
    return get_vect_len(&vect);
}
uint8_t is_vect_same_direction(vect_t * v1, vect_t * v2)
{
    int32_t X1 = v1->e.x - v1->s.x;
    int32_t Y1 = v1->e.y - v1->s.y;
    int32_t X2 = v2->e.x - v2->s.x;
    int32_t Y2 = v2->e.y - v2->s.y;
    if (X1 * X2 + Y1 * Y2 >= 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/*
 *	get_v2x_ang:	��ȡ������X��ļн�
 *	p_vect:			����ָ��
 *	����ֵ��		������X��ļнǡ�0-360�㣩(ǰ�����)
 */
float get_v2x_ang(vect_t *p_vect)
{
	int32_t x = p_vect->e.x - p_vect->s.x;		/*����������X����*/
	int32_t y = p_vect->e.y - p_vect->s.y;		/*����������Y����*/
	
	/*�жϽǶ����䣬���������˷�������X��ĽǶ�*/
	if (x == 0 && y == 0)
	{
		return 0;
	}								
	else if (y>=0)
	{
		return  my_acos((float)x / sqrt(square(x)+square(y)));
	}
	else
	{
		return 360 - my_acos((float)x / sqrt(square(x)+square(y)));
	}
}

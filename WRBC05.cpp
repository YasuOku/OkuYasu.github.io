
#include "WRBC05.h"


int main()
{
	const double DOC = 49.82;				/* ���S����[mm] */
	const int teeth_num = 50;			/* ����[��] */
	const double module = 1.0;			/* ���W���[�� */
	const double PA = M_PI / 9.0;		/* ���͊p��/9[rad]  (20[deg]) */
	const double shrink = 12.0 / M_PI;	/* ���k�� ��3.82 */
	const int div_num = 10000;			/* ������Ԃ̕����� */

	cout << "���S����[mm]:	" << DOC << endl;
	cout << "����[��]:	" << teeth_num << endl;
	cout << "���W���[��:	" << module << endl;
	cout << "���͊p:		" << PA << endl;
	cout << "���k��:		" << shrink << endl;
	cout << "����������:	" << div_num << "\n" << endl;

	design_driving_gear(DOC, shrink, module, PA, teeth_num, div_num);
	design_driven_gear(DOC, shrink, module, PA, teeth_num, div_num);

	getchar();	//�~�߂�
	return 0;
}

/* -------------------------------------------------------
�֐���:
	Speed
����:
	�쓮���Ԃɑ΂���]�����Ԃ̊p���x�����߂�֐��B
	�쓮���Ԃ�angle[rad]�̏ꍇ�̏]�����Ԃ̊p���x��Ԃ��B
����:
	shrink: �e���x��̌W��
	angle:  ���ڂ��Ă���p�x
�߂�l:
	�Ȃ�
------------------------------------------------------- */
double Speed(const double shrink, const double angle)
{
	double radius = 0;
	radius = 1.0 + cos(angle) / shrink;
	return radius;
}

/* -------------------------------------------------------
�֐���:
	DrivingGear_rad
����:
	�������(�p�x)�̋쓮���Ԃ̔��a��Ԃ��֐��B
	(����~�`���ԂȂ̂ŁA�p�x�ɂ���Ĕ��a���Ⴄ)
����:
	dist_of_center: �쓮���ԂƏ]�����Ԃ̒��S�ԋ���
	shrink:         �e���x��̌W��
	angle:          ���ڂ��Ă���p�x
�߂�l:
	angle[rad]�����̏ꍇ�̋쓮���Ԃ̌a
------------------------------------------------------- */
double DrivingGear_rad(const double dist_of_center, const double shrink, const double angle)
{
	double speed = Speed(shrink, angle);
	return dist_of_center * speed / (1.0 + speed);
}

/* -------------------------------------------------------
�֐���:
	DrivenGear_rad
����:
	�������(�p�x)�̏]�����Ԃ̔��a��Ԃ��֐��B
	(����~�`���ԂȂ̂ŁA�p�x�ɂ���Ĕ��a���Ⴄ)
����:
	dist_of_center: �쓮���ԂƏ]�����Ԃ̒��S�ԋ���
	shrink:         �e���x��̌W��
	angle:          ���ڂ��Ă���p�x
�߂�l:
	angle[rad]�����̏ꍇ�̏]�����Ԃ̌a
------------------------------------------------------- */
double DrivenGear_rad(const double dist_of_center, const double shrink, const double angle)
{
	double speed = Speed(shrink, angle);
	return dist_of_center / (1.0 + speed);
}

/* -------------------------------------------------------
�֐���:
	calcPerimeter_driving
����:
	�쓮���C�Ԏ����Z�o�֐��B
	���C�Ԃ̎����𐔒l�ϕ��ŋ��߂�B
	������Ԃ͗]���藝���狁�߂�B
����:
	dist_of_center:	���Ԃ̒��S�ԋ���
	shrink:         �e���x��̌W��
	div_num:        �`��_��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
double calcPerimeter_driving(double dist_of_center, double shrink, int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		accum_perimeter += sqrt(	/* �]���藝���A�������(�����j���Z�o */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}

/* -------------------------------------------------------
�֐���:
	calcPerimeter_driving
����:
	�]�����C�Ԏ����Z�o�֐��B
	���C�Ԃ̎����𐔒l�ϕ��ŋ��߂�B
	������Ԃ͗]���藝���狁�߂�B
����:
	dist_of_center:	���Ԃ̒��S�ԋ���
	shrink:         �e���x��̌W��
	div_num:        �`��_��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
double calcPerimeter_driven(double dist_of_center, double shrink, int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		accum_perimeter += sqrt(	/* �]���藝���A�������(�����j���Z�o */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}

/* -------------------------------------------------------
�֐���:
	calcPerimeter_driving
����:
	�쓮���Ԃ̎��ʒu�E�������̍쐬�֐��B
����:
	dist_of_center:	 ���Ԃ̒��S�ԋ���
	shrink:          �e���x��̌W��
	total_perimeter: �쓮���C�Ԃ̎���
	tooth_num:		 ����
	div_num:         �`��_��
	tooth_angles:    ���C�ԏ�̎��̌����̔z��
	tooth_points:    ���C�ԏ�̎��𐶂₷�ʒu�̔z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void calcToothAnglesAndPoints_driving(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = 0.0;	/* ���������쓮���ԂƏ]�����Ԃ��Ⴄ */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		if (accum_perimeter >= (total_perimeter / tooth_num) * teeth_index + pitch_offset)
		{	/* ���������̈ʒu�𒴂�����z��Ɏ��Ԃ̒��S�Ɗp�x���i�[���郋�[�v */
			/* ��~(�s�b�`�~)��̎��̒��S�_ */
			double x_std = radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double y_std = radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* �ڐ��̃x�N�g�� */
			double vx =
				radius2 * cos(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double vy =
				radius2 * sin(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* �@���x�N�g�� */		/* �ʓ|�Ȃ̂ŁA�P�ʃx�N�g���ɂ͂��ĂȂ���B�P�ʃx�N�g������Ȃ��Ă�����Ȃ����ˁB */
			double nvx = vy;
			double nvy = -1.0 * vx;

			/* ���̊p�x�� */
			double theta;
			if (nvx >= 0.0 && nvy >= 0.0)		/* ��1�ی� */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy >= 0.0)	/* ��2�ی� */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy < 0.0)	/* ��3�ی� */
				theta = M_PI - asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx >= 0.0 && nvy < 0.0)	/* ��4�ی� */
				theta = 2.0 * M_PI + asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			/* ��~��̎��̈ʒu�Ɗp�x��ۑ� */
			tooth_points.push_back(coord2d(x_std, y_std));	/* ���̈ʒu��z��ɕۑ� */
			tooth_angles.push_back(theta);					/* ���̊p�x��z��ɕۑ� */

			teeth_index++;
		}
		accum_perimeter += sqrt(	/* �]���藝���A�������(�����j���Z�o */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2 * M_PI / div_num)
		);
	}

}
/* -------------------------------------------------------
�֐���:
	calcToothAnglesAndPoints_driven
����:
	�]�����Ԃ̎��ʒu�E�������̍쐬�֐��B
	�쓮���Ԃ̎��Ɗ��ݍ����悤�ɁA���̈ʒu�̓s�b�`�̔��������i�߂�B
����:
	dist_of_center:	 ���Ԃ̒��S�ԋ���
	shrink:          �e���x��̌W��
	total_perimeter: �쓮���C�Ԃ̎���
	tooth_num:		 ����
	div_num:         �`��_��
	tooth_angles:    ���C�ԏ�̎��̌����̔z��
	tooth_points:    ���C�ԏ�̎��𐶂₷�ʒu�̔z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void calcToothAnglesAndPoints_driven(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = (total_perimeter / tooth_num) / 2.0;	/* ���̈ʒu���s�b�`�̔����������炷 */		/* ���������쓮���ԂƏ]�����Ԃ��Ⴄ */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		if (accum_perimeter >= (total_perimeter / tooth_num) * teeth_index + pitch_offset)
		{	/* ���������̈ʒu�𒴂�����z��Ɏ��Ԃ̒��S�Ɗp�x���i�[���郋�[�v */
			/* ��~(�s�b�`�~)��̎��̒��S�_ */
			double x_std = radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double y_std = radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* �ڐ��̃x�N�g�� */
			double vx =
				radius2 * cos(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double vy =
				radius2 * sin(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* �@���x�N�g�� */		/* �ʓ|�Ȃ̂ŁA�P�ʃx�N�g���ɂ͂��ĂȂ���B�P�ʃx�N�g������Ȃ��Ă�����Ȃ����ˁB */
			double nvx = vy;
			double nvy = -1.0 * vx;

			/* ���̊p�x�� */
			double theta;
			if (nvx >= 0.0 && nvy >= 0.0)		/* ��1�ی� */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy >= 0.0)	/* ��2�ی� */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy < 0.0)	/* ��3�ی� */
				theta = M_PI - asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx >= 0.0 && nvy < 0.0)	/* ��4�ی� */
				theta = 2.0 * M_PI + asin(nvy / sqrt(nvx * nvx + nvy * nvy));

			/* ��~��̎��̈ʒu�Ɗp�x��ۑ� */
			tooth_points.push_back(coord2d(x_std, y_std));	/* ���̈ʒu��z��ɕۑ� */
			tooth_angles.push_back(theta);					/* ���̊p�x��z��ɕۑ� */
			teeth_index++;
		}
		accum_perimeter += sqrt(	/* �]���藝���A�������(�����j���Z�o */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2 * M_PI / div_num)
		);
	}

}

/* -------------------------------------------------------
�֐���:
	storeGear_driving
����:
	�쓮���ԍ쐬���z��Ɋi�[����֐��B
	�E index�Ԗڂ̎��ɒ��ڂ��A�O��̎��̈ʒu�ƍ��킹��3�_���牼�z�I�Ȋ�~���S���v�Z����
	�Eindex�Ԗڂ̎��̎���̓_���쐬����B
	�Eindex�Ԗڂ̎������_�֕��s�ړ�����
	�Eindex�Ԗڂ̎������̌����ɍ��킹�ĉ�]�ړ�����
	�Eindex�Ԗڂ̎�����~��֕��s�ړ�����
	�Eindex�Ԗڂ̎��̎���̓_���i�[����
	�E(index+1)�Ԗڂ̎��̏����ֈڍs
����:
	module:         ���W���[��
	tooth_num:      ����
	pressure_angle: ���͊p
	tooth_angles:   ���C�ԏ�̎��̌����̔z��
	tooth_points:   ���C�ԏ�̎��𐶂₷�ʒu�̔z��
	gear_plots:     ���Ԃ̎���̓_�̔z��(�o�͗p)
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void storeGear_driving(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots)
{
	for (int index = 0; index < tooth_num; index++) {
		/* index�Ԗڂ̎��ɒ��ڂ��� */
		vector<coord2d> plots;
		coord2d r_c = calc_center(	/* �ߖT��3�̎����牼�z�̊�~�̒��S */
			tooth_points.at((index + tooth_num - 1) % tooth_num),
			tooth_points.at(index),
			tooth_points.at((index + 1) % tooth_num));
		double diameter = 2.0 * sqrt(	/* ���z�̊�~���a*/
			pow(tooth_points.at(index).getx() - r_c.getx(), 2) +
			pow(tooth_points.at(index).gety() - r_c.gety(), 2));
		/* ��̎����쐬 */
		thooth_drawing(diameter, module, pressure_angle, 0.5, tooth_points.at(index), plots);
		double dx, dy;
		/* �������_�ɕ��s�ړ� */
		dx = -1.0 * diameter / 2.0;
		dy = 0.0;
		paramove(dx, dy, plots);
		/* �����s�b�`�~���̖@�������Ɍ����ĉ�] */
		rotmove(tooth_angles.at(index), plots);
		/* �������_�Ƀs�b�`�~����ɕ��s�ړ� */
		dx = tooth_points.at(index).getx();
		dy = tooth_points.at(index).gety();
		paramove(dx, dy, plots);
		///* �쐬����������y���𒆐S�ɔ��] */		/* ���������쓮���ԂƏ]�����Ԃ��Ⴄ */
		//mirroring_y(plots);						/* ���������쓮���ԂƏ]�����Ԃ��Ⴄ */
		/* �쐬�����������i�[ */
		gear_plots.insert(gear_plots.end(), plots.begin(), plots.end());
		plots.clear();
	}
}

/* -------------------------------------------------------
�֐���:
	storeGear_driven
����:
	�]�����ԍ쐬���z��Ɋi�[����֐��B
	�E index�Ԗڂ̎��ɒ��ڂ��A�O��̎��̈ʒu�ƍ��킹��3�_���牼�z�I�Ȋ�~���S���v�Z����
	�Eindex�Ԗڂ̎��̎���̓_���쐬����B
	�Eindex�Ԗڂ̎������_�֕��s�ړ�����
	�Eindex�Ԗڂ̎������̌����ɍ��킹�ĉ�]�ړ�����
	�Eindex�Ԗڂ̎�����~��֕��s�ړ�����
	�Eindex�Ԗڂ̎��̎���̓_���i�[����
	�E(index+1)�Ԗڂ̎��̏����ֈڍs
����:
	module:         ���W���[��
	tooth_num:      ����
	pressure_angle: ���͊p
	tooth_angles:   ���C�ԏ�̎��̌����̔z��
	tooth_points:   ���C�ԏ�̎��𐶂₷�ʒu�̔z��
	gear_plots:     ���Ԃ̎���̓_�̔z��(�o�͗p)
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void storeGear_driven(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots)
{
	for (int index = 0; index < tooth_num; index++) {
		/* index�Ԗڂ̎��ɒ��ڂ��� */
		vector<coord2d> plots;
		coord2d r_c = calc_center(	/* �ߖT��3�̎����牼�z�̊�~�̒��S */
			tooth_points.at((index + tooth_num - 1) % tooth_num),
			tooth_points.at(index),
			tooth_points.at((index + 1) % tooth_num));
		double diameter = 2.0 * sqrt(	/* ���z�̊�~���a*/
			pow(tooth_points.at(index).getx() - r_c.getx(), 2) +
			pow(tooth_points.at(index).gety() - r_c.gety(), 2));
		/* ��̎����쐬 */
		thooth_drawing(diameter, module, pressure_angle, 0.5, tooth_points.at(index), plots);
		double dx, dy;
		/* �������_�ɕ��s�ړ� */
		dx = -1.0 * diameter / 2.0;
		dy = 0.0;
		paramove(dx, dy, plots);
		/* �����s�b�`�~���̖@�������Ɍ����ĉ�] */
		rotmove(tooth_angles.at(index), plots);
		/* �������_�Ƀs�b�`�~����ɕ��s�ړ� */
		dx = tooth_points.at(index).getx();
		dy = tooth_points.at(index).gety();
		paramove(dx, dy, plots);
		/* �쐬����������y���𒆐S�ɔ��] */
		mirroring_y(plots);					/* ���������쓮���ԂƏ]�����Ԃ��Ⴄ */
		/* �쐬�����������i�[ */
		gear_plots.insert(gear_plots.end(), plots.begin(), plots.end());
		plots.clear();
	}
}

/* -------------------------------------------------------
�֐���:
	drawGear_driving
����:
	�쓮���ԕ`��֐��B
����:
	gear_plots:	���Ԃ̎���̓_�̔z��
	offset_x:   x�������̃I�t�Z�b�g�l
����:
	FreeCAD��dat�t�@�C����ǂݍ��߂邪�Ax����0�ȏ�łȂ���΂Ȃ�Ȃ��B
	�����offset_x�ɓK���Ȑ��l�����Ă�����FreeCAD�œǂݍ��߂�悤�ɂ���B
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void drawGear_driving(vector<coord2d>& gear_plots, double offset_x)
{
	ofstream ofs_dat("driving_gear.dat");
	ofs_dat << "driving_gear.dat" << endl;
	ofstream ofs_csv("driving_gear.csv");
	vector<coord2d>::iterator p = gear_plots.begin();
	while (p != gear_plots.end())
	{
		ofs_dat << p->getx() + offset_x << " " << p->gety() << endl;
		ofs_csv << p->getx() << "," << p->gety() << ",0.0" << endl;
		p++;
	}
	ofs_dat.close();
	ofs_csv.close();
}

/* -------------------------------------------------------
�֐���:
	drawGear_driven
����:
	�쓮���ԕ`��֐��B
����:
	gear_plots:	���Ԃ̎���̓_�̔z��
	offset_x:   x�������̃I�t�Z�b�g�l
����:
	FreeCAD��dat�t�@�C����ǂݍ��߂邪�Ax����0�ȏ�łȂ���΂Ȃ�Ȃ��B
	�����offset_x�ɓK���Ȑ��l�����Ă�����FreeCAD�œǂݍ��߂�悤�ɂ���B
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void drawGear_driven(vector<coord2d>& gear_plots, double offset_x)
{
	ofstream ofs_dat("driven_gear.dat");
	ofs_dat << "driven_gear.dat" << endl;
	ofstream ofs_csv("driven_gear.csv");
	vector<coord2d>::iterator p = gear_plots.begin();
	while (p != gear_plots.end())
	{
		ofs_dat << p->getx() + offset_x << " " << p->gety() << endl;
		ofs_csv << p->getx() << "," << p->gety() << ",0.0" << endl;
		p++;
	}
	ofs_dat.close();
	ofs_csv.close();
}

/* -------------------------------------------------------
�֐���:
	design_driving_gear
����:
	�쓮���ԍ쐬�����̃T�u���[�`���B
	�E���̈ʒu�ƌ����̔z����쐬����
	�E���̍쐬�Ɣz�u���Ď��Ԃ����
	�E���Ԃ��t�@�C��(dat)�ɏo�͂���
����:
	dist_of_center:	���Ԃ̒��S�ԋ���
	coefficient:    �e���x��̌W��
	module:	        ���W���[��
	pressure_angle: ���͊p
	tooth_num:		����
	div_num:        �`��_��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void design_driving_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num)
{
	vector<coord2d> centers;
	vector<double> angles;
	vector<coord2d> gears;
	const double total_perimeter = calcPerimeter_driving(dist_of_center, coefficient, div_num);
	cout << "<�쓮����>" << endl;
	cout << "		total perimeter:		" << total_perimeter << endl;
	cout << "		tooth number (computed):	" << total_perimeter / M_PI / module << endl;
	cout << "		tooth number:			" << tooth_num << endl;
	cout << "		tooth number diffs:		" << abs(tooth_num - total_perimeter / M_PI / module) << endl;

	calcToothAnglesAndPoints_driving(dist_of_center, coefficient, total_perimeter, tooth_num, div_num, angles, centers);
	storeGear_driving(module, tooth_num, pressure_angle, angles, centers, gears);
	drawGear_driving(gears, 100);
}
/* -------------------------------------------------------
�֐���:
	design_driven_gear
����:
	�]�����ԍ쐬�����̃T�u���[�`���B
	�E���̈ʒu�ƌ����̔z����쐬����
	�E���̍쐬�Ɣz�u���Ď��Ԃ����
	�E���Ԃ��t�@�C��(dat)�ɏo�͂���B
����:
	perimeter:
	module: ���W���[��
	pressure_angle: ���͊p
	tooth_num: ����
	div_num: ���C�Ԃ��v�Z����ۂ̊O���̕����� (�v�Z�ׂ̍���)
	plots: �C���{�����[�g����̓_���i�[����z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void design_driven_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num)
{
	vector<coord2d> centers;
	vector<double> angles;
	vector<coord2d> gears;
	const double total_perimeter = calcPerimeter_driven(dist_of_center, coefficient, div_num);
	cout << "<�]������>" << endl;
	cout << "		total perimeter:		" << total_perimeter << endl;
	cout << "		tooth number (computed):	" << total_perimeter / M_PI / module << endl;
	cout << "		tooth number:			" << tooth_num << endl;
	cout << "		tooth number diffs:		" << abs(tooth_num - total_perimeter / M_PI / module) << endl;

	calcToothAnglesAndPoints_driven(dist_of_center, coefficient, total_perimeter, tooth_num, div_num, angles, centers);
	storeGear_driven(module, tooth_num, pressure_angle, angles, centers, gears);
	drawGear_driven(gears, 100);
}
/* -------------------------------------------------------
�֐���:
	thooth_drawing
����:

	�_�ׂ̍����͊֐����Œ�`����B
	�n���x���v���X�����Ɍ����ĐQ����Ԃ̃C���{�����[�g�����쐬����B
	���̃C���{�����[�g����̓_��z��ɋl�߂�֐��B
	�_�͉��L�̎菇�ō쐬����B
	�@����~��̓_��z��Ɋi�[����B(�������A��b�~�̒��a������~�̒��a�����傫���ꍇ�Ɍ���)
	�A�������̃C���{�����[�g�Ȑ���̓_��z��Ɋi�[����
	�B����~��̓_��z��Ɋi�[����
	�C��L�@�`�B�ŃC���{�����[�g���̉��������ł����̂ŁA�쐬�����_�𔽓]���ď㔼���̎���̓_��z��ɋl�߂�
����:
	img_std_diameter: ���z�W���~���a
	module: ���W���[��
	pressure_angle: ���͊p
	vct: ���Ԃ��͂₷�����ւ̃x�N�g���B�x�N�g���̑傫���ɈӖ��͂Ȃ��B
	plots: �C���{�����[�g����̓_���i�[����z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void thooth_drawing(double img_std_diameter, double module, double pressure_angle, double clearance, coord2d vct, vector<coord2d>& plots)
{
	const double plot_num = 30;	/* �`��_�� */
	const coord2d origin_point(0.0, 0.0);	/* ���_ */
	double involute_start_angle;
	const double involute_end_angle =		/* x������Ƃ��ĉ��ɉ��x[rad]�ŃC���{�����[�g�Ȑ�������~�ɒB���邩 */
		acos(
			(img_std_diameter / 2.0 * cos(pressure_angle)) /
			(img_std_diameter / 2.0 + module));

	coord2d tmp_point;
	if (img_std_diameter * cos(pressure_angle) / 2.0 > img_std_diameter / 2.0 - (1.0 + clearance) * module) {
		involute_start_angle = 	/* x������Ƃ��ĉ��ɉ��x[rad]����C���{�����[�g����`���n�߂邩 */
			TT_to_Angle(img_std_diameter, module, pressure_angle, 0.9, img_std_diameter * cos(pressure_angle));
		/* ����~��̓_��1�_�`�� */
		coord2d point = coord2d(
			(img_std_diameter / 2.0 - (1.0 + clearance) * module) * cos(-1.0 * involute_start_angle),
			(img_std_diameter / 2.0 - (1.0 + clearance) * module) * sin(-1.0 * involute_start_angle)
		);
		plots.push_back(point);
	}
	else {
		involute_start_angle = 	/* x������Ƃ��ĉ��ɉ��x[rad]����C���{�����[�g����`���n�߂邩 */
			TT_to_Angle(img_std_diameter, module, pressure_angle, 0.9, img_std_diameter - (1.0 + clearance) * module);
	}

	/* �C���{�����[�g�Ȑ�������`�� */
	vector<coord2d> tmp_points;
	double dist = distance(origin_point, origin_point);
	for (double angle = 0.0;
		angle <= involute_end_angle;				/* �������͂��Ȃ񂾂��ǁA���̏������Ǝ���Ɗ��炩�Ɍq����Ȃ��񂾂�ˁB�v�Z�̌덷����... */
		//dist < img_std_diameter / 2.0 + module;	/* �u���攼�a�ɒB����܂Łv�������ɂ��Ă�����ς��v���Ȃ��B�����ǂ����悤���Ȃ���... */
		angle += involute_end_angle / plot_num)
	{
		tmp_point = involute_drawing(angle, pressure_angle, img_std_diameter);	/* �p�xangle�̃C���{�����[�g�Ȑ���̓_��`�� */
		rotmove(-1.0 * involute_start_angle, tmp_point);						/* �p�xinvolute_start_angle�������u���Ɂv������ */
		plots.push_back(tmp_point);
		dist = distance(tmp_point, origin_point);
	}
	double tmp = distance(tmp_point, origin_point);
	/* �������𔽓]���ď㔼����`�� */
	for (int index = plots.size() - 1; index >= 0; index--)
	{
		coord2d additional(plots.at(index).getx(), -1.0 * plots.at(index).gety());
		plots.push_back(additional);
	}

}


/* -------------------------------------------------------
�֐���:
	involute_drawing
����:
	�C���{�����[�g�Ȑ���̓_�̍쐬�֐��B
����:
	angle: �u���̐�[�v�u��b�~���S�v�u��b�~�Ǝ��̐ړ_�v�̂Ȃ��p
	pressure_angle: ���͊p
	std_diameter: ��~���a
�߂�l:
	�C���{�����[�g�Ȑ���̍��W
------------------------------------------------------- */
coord2d involute_drawing(double angle, double pressure_angle, double std_diameter)
{
	return coord2d(
		std_diameter * cos(pressure_angle) * cos(inv(angle)) / cos(angle) / 2.0,
		std_diameter * cos(pressure_angle) * sin(inv(angle)) / cos(angle) / 2.0
	);
}


/* -------------------------------------------------------
�֐���:
	paramove
����:
	���W�̕��s�ړ��֐��B
	�����œn���ꂽ���W�z�񂩂�v�f���擾����B
	(dx, dy)���s�ړ����������W���쐬�������v�f�Ɋi�[���Ȃ����B
����:
	dx: x�������̈ړ�����
	dy: y�������̈ړ�����
	plots: x-y���W�̈ʒu���i�[����z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void paramove(
	double dx,
	double dy,
	vector<coord2d>& plots)
{
	vector<coord2d>::iterator p = plots.begin();

	while (p != plots.end())
	{
		p->setx(p->getx() + dx);
		p->sety(p->gety() + dy);
		p++;
	}
}

/* -------------------------------------------------------
�֐���:
	rotmove
����:
	���W�̉�]�ړ��֐��B
	�����œn���ꂽ���W�z�񂩂�v�f���擾����B
	theta��]���������W���쐬�������v�f�Ɋi�[���Ȃ����B
����:
	theta: ��]�p�x[rad]
	plots: x-y���W�̈ʒu���i�[����z��
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void rotmove(double theta, vector<coord2d>& plots)
{
	vector<coord2d>::iterator p = plots.begin();

	while (p != plots.end())
	{
		coord2d tmp(p->getx(), p->gety());
		p->setx(cos(theta) * tmp.getx() - sin(theta) * tmp.gety());
		p->sety(sin(theta) * tmp.getx() + cos(theta) * tmp.gety());
		p++;
	}
}

/* -------------------------------------------------------
�֐���:
	rotmove
����:
	���W�̉�]�ړ��֐��B
	�����œn���ꂽ���W��theta��]���������W�������Ɋi�[���Ȃ����B
����:
	theta: ��]�p�x[rad]
	plot: x-y���W
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void rotmove(double theta, coord2d& plot)
{
	double tmp_x = plot.getx();
	double tmp_y = plot.gety();
	plot.setx(cos(theta) * tmp_x - sin(theta) * tmp_y);
	plot.sety(sin(theta) * tmp_x + cos(theta) * tmp_y);
}

/* -------------------------------------------------------
�֐���:
	mirroring_y
����:
	���W�̔��](�~���[)�֐��B
	�����œn���ꂽ���W�z�񂩂�v�f���擾����B
	y���ɑ΂��đΏ̂ȍ��W���쐬�������v�f�Ɋi�[���Ȃ����B
����:
	plots: x-y���W�̈ʒu���i�[����z��B
�߂�l:
	�Ȃ�
------------------------------------------------------- */
void mirroring_y(vector<coord2d>& plots)
{
	vector<coord2d>::iterator p = plots.begin();

	while (p != plots.end())
	{
		p->setx(-1.0 * p->getx());
		p++;
	}
}


/* -------------------------------------------------------
�֐���:
	TT_to_Angle
����:
	��b�~�ɑ΂��鎕����Ԃ��֐�
����:
	std_diameter:��~���a
	module: ���W���[��
	pressure_angle: ���͊p
	target_diameter: �p�x�����߂������̍���(���a)
�߂�l:
	��b�~��̎����̂Ȃ��p�̔���
------------------------------------------------------- */
double TT_to_Angle(double std_diameter, double module, double pressure_angle, double backlash, double target_diameter)
{
	double target_angle = 0;
	const double phi = acos(std_diameter * cos(pressure_angle) / target_diameter);	/* ���̒����� �Ȃ��p */
	const double S_std = M_PI * module / 2.0;										/* ��~��̎���(�W�����ƃ�m/2�Ȃ񂾂��Ă�) */
	const double S_target =															/* target���� */
		target_diameter *
		(S_std / std_diameter + inv(pressure_angle) - inv(phi));
	target_angle = (S_target / 2.0) / (target_diameter / 2.0);						/* (�����̔���) / (target���a) / 2 */
	target_angle *= backlash;														/* �o�b�N���b�V�Ƃ��Ď�����95%�ɂ��Ă����� */

	return target_angle;
}

/* -------------------------------------------------------
�֐���:
	TT_b
����:
	�C���{�����[�g�p�����߂�֐�
����:
	angle:
�߂�l:
	�C���{�����[�g�p
------------------------------------------------------- */
double inv(double angle)	/* �p */
{
	double inva = tan(angle) - angle;
	return inva;
}

/* -------------------------------------------------------
�֐���:
	calc_center
����:
	 3�_��ʂ�~�̒��S�����߂�
����:
	p1: ��1�_
	p2: ��2�_
	p3: ��3�_
�߂�l:
	�~�̒��S�̍��W
------------------------------------------------------- */
coord2d calc_center(coord2d& p1, coord2d& p2, coord2d& p3)
{
	double X = 0.0;
	double Y = 0.0;

	double g1 = (p1.getx() - p2.getx()) / (p1.gety() - p2.gety());
	double g2 = (p2.getx() - p3.getx()) / (p2.gety() - p3.gety());
	double x1 = (p1.getx() + p2.getx()) / 2.0;
	double y1 = (p1.gety() + p2.gety()) / 2.0;
	double x2 = (p2.getx() + p3.getx()) / 2.0;
	double y2 = (p2.gety() + p3.gety()) / 2.0;
	X = (g1 * x1 - g2 * x2 + y1 - y2) / (g1 - g2);
	Y = -1.0 * g1 * (X - x1) + y1;
	return coord2d(X, Y);
}

/* -------------------------------------------------------
�֐���:
	distance
����:
	��1�_�Ƒ�2�_�̋��������߂�֐�
����:
	p1: ��1�_
	p2: ��2�_
�߂�l:
	��1�_�Ƒ�2�_�̋���
------------------------------------------------------- */
double distance(coord2d p1, coord2d p2)
{
	return sqrt(pow(p1.getx() - p2.getx(), 2.0) + pow(p1.gety() - p2.gety(), 2.0));
}
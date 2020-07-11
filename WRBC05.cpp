
#include "WRBC05.h"


int main()
{
	const double DOC = 50;				/* 中心距離[mm] */
	const int teeth_num = 25;			/* 歯数[個] */
	const double shrink = 12.0 / M_PI;	/* 圧縮率 ≒3.82 */
	const int div_num = 1000;			/* 微小区間の分割数 */
	const double module = 2.0;			/* モジュール */
	const double PA = M_PI / 9.0;		/* 圧力角π/9[rad]  (20[deg]) */

	design_driving_gear(DOC, shrink, module, PA, teeth_num, div_num);
	design_driven_gear(DOC, shrink, module, PA, teeth_num, div_num);

	return 0;
}

double Speed(const double shrink, const double angle)
{
	double radius = 0;
	radius = 1.0 + cos(angle) / shrink;
	return radius;
}
double DrivingGear_rad(const double perimeter, const double speed)
{
	return perimeter * speed / (1.0 + speed);
}
double DrivenGear_rad(const double perimeter, const double speed)
{
	return perimeter / (1.0 + speed);
}
/* 駆動摩擦車周長算出関数 */
double calcPerimeter_driving(double dist_of_center, double shrink, int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index) / div_num));
		radius2 = DrivingGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num));
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}
/* 従動摩擦車周長算出関数 */
double calcPerimeter_driven(double dist_of_center, double shrink,  int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index) / div_num));
		radius2 = DrivenGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num));
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}
/* 駆動歯車の歯位置・歯向きの作成関数 */
void calcToothAnglesAndPoints_driving(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = 0.0;	/* ←ここが駆動歯車と従動歯車が違う */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index) / div_num));
		radius2 = DrivingGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num));
		if (accum_perimeter >= (total_perimeter / tooth_num) * teeth_index + pitch_offset)
		{	/* 周長が歯の位置を超えたら配列に歯車の中心と角度を格納するループ */
			/* 基準円(ピッチ円)上の歯の中心点 */
			double x_std = radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double y_std = radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* 接線のベクトル */
			double vx =
				radius2 * cos(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double vy =
				radius2 * sin(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* 法線ベクトル */		/* 面倒なので、単位ベクトルにはしてないよ。単位ベクトルじゃなくても困らないしね。 */
			double nvx = vy;
			double nvy = -1.0 * vx;

			/* 歯の角度Θ */
			double theta;
			if (nvx >= 0.0 && nvy >= 0.0)		/* 第1象限 */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy >= 0.0)	/* 第2象限 */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy < 0.0)	/* 第3象限 */
				theta = M_PI - asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx >= 0.0 && nvy < 0.0)	/* 第4象限 */
				theta = 2.0 * M_PI + asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			/* 基準円状の歯の位置と角度を保存 */
			tooth_points.push_back(coord2d(x_std, y_std));	/* 歯の位置を配列に保存 */
			tooth_angles.push_back(theta);					/* 歯の角度を配列に保存 */

			teeth_index++;
		}
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2 * M_PI / div_num)
		);
	}

}
/* 従動駆動歯車の歯位置・歯向きの作成関数 */
void calcToothAnglesAndPoints_driven(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = (total_perimeter / tooth_num) / 2.0;	/* ←ここが駆動歯車と従動歯車が違う */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index) / div_num));
		radius2 = DrivenGear_rad(dist_of_center, Speed(shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num));
		if (accum_perimeter >= (total_perimeter / tooth_num) * teeth_index + pitch_offset)
		{	/* 周長が歯の位置を超えたら配列に歯車の中心と角度を格納するループ */
			/* 基準円(ピッチ円)上の歯の中心点 */
			double x_std = radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double y_std = radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* 接線のベクトル */
			double vx =
				radius2 * cos(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * cos(2.0 * M_PI * (double)(index) / div_num);
			double vy =
				radius2 * sin(2.0 * M_PI * (double)(index + 1.0) / div_num) -
				radius1 * sin(2.0 * M_PI * (double)(index) / div_num);
			/* 法線ベクトル */		/* 面倒なので、単位ベクトルにはしてないよ。単位ベクトルじゃなくても困らないしね。 */
			double nvx = vy;
			double nvy = -1.0 * vx;

			/* 歯の角度Θ */
			double theta;
			if (nvx >= 0.0 && nvy >= 0.0)		/* 第1象限 */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy >= 0.0)	/* 第2象限 */
				theta = acos(nvx / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx < 0.0 && nvy < 0.0)	/* 第3象限 */
				theta = M_PI - asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			else if (nvx >= 0.0 && nvy < 0.0)	/* 第4象限 */
				theta = 2.0 * M_PI + asin(nvy / sqrt(nvx * nvx + nvy * nvy));
			/* 基準円状の歯の位置と角度を保存 */
			tooth_points.push_back(coord2d(x_std, y_std));	/* 歯の位置を配列に保存 */
			tooth_angles.push_back(theta);					/* 歯の角度を配列に保存 */

			teeth_index++;
		}
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2 * M_PI / div_num)
		);
	}

}
/* 駆動歯車作成関数 */
void storeGear_driving(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots)
{
	for (int index = 0; index < tooth_num; index++) {
		/* index番目の歯に注目する */
		vector<coord2d> plots;
		coord2d r_c = calc_center(	/* 近傍の3つの歯から仮想の基準円の中心 */
			tooth_points.at((index + tooth_num - 1) % tooth_num),
			tooth_points.at(index),
			tooth_points.at((index + 1) % tooth_num));
		double diameter = 2.0 * sqrt(	/* 仮想の基準円半径*/
			pow(tooth_points.at(index).getx() - r_c.getx(), 2) +
			pow(tooth_points.at(index).gety() - r_c.gety(), 2));
		/* 一つの歯を作成 */
		thooth_drawing(diameter, module, pressure_angle, tooth_points.at(index), plots);
		double dx, dy;
		/* 歯を原点に平行移動 */
		dx = -1.0 * diameter / 2.0;
		dy = 0.0;
		paramove(dx, dy, plots);
		/* 歯をピッチ円周の法線方向に向けて回転 */
		rotmove(tooth_angles.at(index), plots);
		/* 歯を原点にピッチ円周上に平行移動 */
		dx = tooth_points.at(index).getx();
		dy = tooth_points.at(index).gety();
		paramove(dx, dy, plots);
		///* 作成した部分をy軸を中心に反転 */		/* ←ここが駆動歯車と従動歯車が違う */
		//mirroring_y(plots);						/* ←ここが駆動歯車と従動歯車が違う */
		/* 作成した部分を格納 */
		gear_plots.insert(gear_plots.end(), plots.begin(), plots.end());
		plots.clear();
	}
}
/* 従動歯車作成関数 */
void storeGear_driven(double module, int tooth_num, double pressure_angle, vector<double>& tooth_angles, vector<coord2d>& tooth_points, vector<coord2d>& gear_plots)
{
	for (int index = 0; index < tooth_num; index++) {
		/* index番目の歯に注目する */
		vector<coord2d> plots;
		coord2d r_c = calc_center(	/* 近傍の3つの歯から仮想の基準円の中心 */
			tooth_points.at((index + tooth_num - 1) % tooth_num),
			tooth_points.at(index),
			tooth_points.at((index + 1) % tooth_num));
		double diameter = 2.0 * sqrt(	/* 仮想の基準円半径*/
			pow(tooth_points.at(index).getx() - r_c.getx(), 2) +
			pow(tooth_points.at(index).gety() - r_c.gety(), 2));
		/* 一つの歯を作成 */
		thooth_drawing(diameter, module, pressure_angle, tooth_points.at(index), plots);
		double dx, dy;
		/* 歯を原点に平行移動 */
		dx = -1.0 * diameter / 2.0;
		dy = 0.0;
		paramove(dx, dy, plots);
		/* 歯をピッチ円周の法線方向に向けて回転 */
		rotmove(tooth_angles.at(index), plots);
		/* 歯を原点にピッチ円周上に平行移動 */
		dx = tooth_points.at(index).getx();
		dy = tooth_points.at(index).gety();
		paramove(dx, dy, plots);
		/* 作成した部分をy軸を中心に反転 */
		mirroring_y(plots);				/* ←ここが駆動歯車と従動歯車が違う */
		/* 作成した部分を格納 */		/* ←ここが駆動歯車と従動歯車が違う */
		gear_plots.insert(gear_plots.end(), plots.begin(), plots.end());
		plots.clear();
	}
}
/* 駆動歯車描画関数 */
void drawGear_driving(vector<coord2d>& gear_plots, double offset_x)
{
	ofstream ofs("driving_gear.dat");
	ofs << "gear_curve_driving.dat" << endl;
	vector<coord2d>::iterator p = gear_plots.begin();
	while (p != gear_plots.end())
	{
		ofs << p->getx() + offset_x << " " << p->gety() << endl;
		p++;
	}
	ofs.close();

}
/* 従動歯車描画関数 */
void drawGear_driven(vector<coord2d>& gear_plots, double offset_x)
{
	ofstream ofs("driven_gear.dat");
	ofs << "gear_curve_driven.dat" << endl;
	vector<coord2d>::iterator p = gear_plots.begin();
	while (p != gear_plots.end())
	{
		ofs << p->getx() + offset_x << " " << p->gety() << endl;
		p++;
	}
	ofs.close();

}
/* 駆動歯車作成関数 */
void design_driving_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num)
{
	vector<coord2d> centers;
	vector<double> angles;
	vector<coord2d> gears;
	const double total_perimeter = calcPerimeter_driving(dist_of_center, coefficient, div_num);
	calcToothAnglesAndPoints_driving(dist_of_center, coefficient, total_perimeter, tooth_num, div_num, angles, centers);
	storeGear_driving(module, tooth_num, pressure_angle, angles, centers, gears);
	drawGear_driving(gears, 100);
}
/* 従動歯車作成関数 */
/* -------------------------------------------------------
関数名:
	design_driven_gear
説明:

	能動歯車を作成すし、配列に詰める。
引数:
	perimeter: 
	module: モジュール
	pressure_angle: 圧力角
	tooth_num: 歯数
	div_num: 摩擦車を計算する際の外周の分割数 (計算の細かさ)
	plots: インボリュート歯上の点を格納する配列
戻り値:
	なし
------------------------------------------------------- */
void design_driven_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num)
{
	vector<coord2d> centers;
	vector<double> angles;
	vector<coord2d> gears;
	const double total_perimeter = calcPerimeter_driven(dist_of_center, coefficient, div_num);
	calcToothAnglesAndPoints_driven(dist_of_center, coefficient, total_perimeter, tooth_num, div_num, angles, centers);
	storeGear_driven(module, tooth_num, pressure_angle, angles, centers, gears);
	drawGear_driven(gears, 100);
}
/* -------------------------------------------------------
関数名:
	thooth_drawing
説明:
	
	点の細かさは関数内で定義する。
	刃先をx軸プラス方向に向けて寝た状態のインボリュート歯を作成する。
	そのインボリュート歯上の点を配列に詰める関数。
	点は下記の手順で作成する。
	①歯底円上の点を配列に格納する。(ただし、基礎円の直径が歯底円の直径よりも大きい場合に限る)
	②下半分のインボリュート曲線上の点を配列に格納する
	③歯先円上の点を配列に格納する
	④上記①～③でインボリュート歯の下半分ができたので、作成した点を反転して上半分の歯上の点を配列に詰める
引数:
	img_std_diameter: 仮想標準円直径
	module: モジュール
	pressure_angle: 圧力角
	vct: 歯車をはやす方向へのベクトル。ベクトルの大きさに意味はない。
	plots: インボリュート歯上の点を格納する配列
戻り値:
	なし
------------------------------------------------------- */
void thooth_drawing(double img_std_diameter, double module, double pressure_angle, coord2d vct, vector<coord2d>& plots)
{
	const double plot_num = 100;	/* 描画点数 */
	const coord2d origin_point(0.0, 0.0);	/* 原点 */
	double involute_start_angle;
	const double involute_end_angle =		/* x軸を基準として下に何度[rad]でインボリュート曲線が歯先円に達するか */
		acos(
			(img_std_diameter / 2.0 * cos(pressure_angle)) /
			(img_std_diameter / 2.0 + module));

	coord2d tmp_point;
	if (img_std_diameter * cos(pressure_angle) < img_std_diameter - 1.25 * module) {
		involute_start_angle = 	/* x軸を基準として下に何度[rad]からインボリュート歯を描き始めるか */
			TT_to_Angle(img_std_diameter, module, pressure_angle, img_std_diameter * cos(pressure_angle));
		/* 歯底円上の点を1点描く */
		coord2d point = coord2d(
			(img_std_diameter / 2.0 - 1.25 * module) * cos(-1.0 * involute_start_angle),
			(img_std_diameter / 2.0 - 1.25 * module) * sin(-1.0 * involute_start_angle)
		);
		plots.push_back(point);
	}
	else {
		involute_start_angle = 	/* x軸を基準として下に何度[rad]からインボリュート歯を描き始めるか */
			TT_to_Angle(img_std_diameter, module, pressure_angle, img_std_diameter - 1.25 * module);
	}

	/* インボリュート曲線部分を描く */
	double dist = distance(origin_point, origin_point);
	for (double angle = 0.0;
		angle <= involute_end_angle;				/* 正しいはずなんだけど、この条件だと歯先と滑らかに繋がらないんだよね。計算の誤差かな... */
		//dist < img_std_diameter / 2.0 + module;	/* 「歯先半径に達するまで」を条件にしてもやっぱり一致しない。もうどうしようもないね... */
		angle += involute_end_angle / plot_num)
	{
		tmp_point = involute_drawing(angle, pressure_angle, involute_start_angle, img_std_diameter);
		plots.push_back(tmp_point);
		dist = distance(tmp_point, origin_point);
	}
	double tmp = distance(tmp_point, origin_point);
	/* 歯先部分を描く */
	const double toothTop_start_angle = atan(tmp_point.gety() / tmp_point.getx());
	for (double angle = toothTop_start_angle; angle <= 0.0; angle += abs(toothTop_start_angle) / plot_num)
	{
		coord2d point = circle_drawing(angle, module, img_std_diameter);
		plots.push_back(point);
	}
	/* 下半分を反射して上半分を描く */
	for (int index = plots.size() - 1; index >= 0; index--)
	{
		coord2d additional(plots.at(index).getx(), -1.0 * plots.at(index).gety());
		plots.push_back(additional);
	}

}


/* インボリュート曲線(下半分) */
coord2d involute_drawing(double angle, double pressure_angle, double start_angle, double std_diameter)
{
	return coord2d(
		std_diameter * cos(pressure_angle) * cos(inv(angle) - start_angle) / cos(angle) / 2.0,
		std_diameter * cos(pressure_angle) * sin(inv(angle) - start_angle) / cos(angle) / 2.0
	);
}

/* 歯先円(下半分) */
coord2d circle_drawing(double angle, double module, double std_diameter)
{
	return coord2d(
		(std_diameter / 2.0 + module) * cos(angle),
		(std_diameter / 2.0 + module) * sin(angle)
	);
}

/* 原点へ移動 */
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

/* 法線方向へ回転 */
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

/* ミラー(y軸に対して反転させる) */
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
関数名:
	TT_to_Angle
説明:
	基礎円に対する歯厚を返す関数
引数:
	std_diameter:基準円半径
	module: モジュール
	pressure_angle: 圧力角
	target_diameter: 角度を求めたい歯の高さ(半径)
戻り値:
	基礎円上の歯厚のなす角の半分
------------------------------------------------------- */
double TT_to_Angle(double std_diameter, double module, double pressure_angle, double target_diameter)
{
	double target_angle = 0;
	const double phi = acos(std_diameter * cos(pressure_angle) / target_diameter);	/* 糸の長さが なす角 */
	const double S_std = M_PI * module / 2.0;	/* 基準円状の歯厚(標準だとπm/2なんだってさ) */
	const double S_target = target_diameter * (S_std / std_diameter + inv(pressure_angle) - inv(phi));	/* target歯厚 */
	target_angle = (S_target / 2.0) / (target_diameter / 2.0);		/* (歯厚の半分) / (target半径) / 2 */
	return target_angle;
}
/* -------------------------------------------------------
関数名:
	TT_b
説明:
	インボリュート角を求める関数
引数:
	angle: 
戻り値:
	インボリュート角
------------------------------------------------------- */
double inv(double angle)	/* 角 */
{
	double inva = tan(angle) - angle;
	return inva;
}

/* -------------------------------------------------------
関数名:
	calc_center
説明:
	 3点を通る円の中心を求める
引数:
	p1: 第1点
	p2: 第2点
	p3: 第3点
戻り値:
	円の中心の座標
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
関数名:
	distance
説明:
	第1点と第2点の距離を求める関数
引数:
	p1: 第1点
	p2: 第2点
戻り値:
	第1点と第2点の距離
------------------------------------------------------- */
double distance(coord2d p1, coord2d p2)
{
	return sqrt(pow(p1.getx() - p2.getx(), 2.0) + pow(p1.gety() - p2.gety(), 2.0));
}
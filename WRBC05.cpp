
#include "WRBC05.h"


int main()
{
	const double DOC = 49.82;				/* 中心距離[mm] */
	const int teeth_num = 50;			/* 歯数[個] */
	const double module = 1.0;			/* モジュール */
	const double PA = M_PI / 9.0;		/* 圧力角π/9[rad]  (20[deg]) */
	const double shrink = 12.0 / M_PI;	/* 圧縮率 ≒3.82 */
	const int div_num = 10000;			/* 微小区間の分割数 */

	cout << "中心距離[mm]:	" << DOC << endl;
	cout << "歯数[個]:	" << teeth_num << endl;
	cout << "モジュール:	" << module << endl;
	cout << "圧力角:		" << PA << endl;
	cout << "圧縮率:		" << shrink << endl;
	cout << "微小分割数:	" << div_num << "\n" << endl;

	design_driving_gear(DOC, shrink, module, PA, teeth_num, div_num);
	design_driven_gear(DOC, shrink, module, PA, teeth_num, div_num);

	getchar();	//止める
	return 0;
}

/* -------------------------------------------------------
関数名:
	Speed
説明:
	駆動歯車に対する従動歯車の角速度を求める関数。
	駆動歯車がangle[rad]の場合の従動歯車の角速度を返す。
引数:
	shrink: 各速度比の係数
	angle:  注目している角度
戻り値:
	なし
------------------------------------------------------- */
double Speed(const double shrink, const double angle)
{
	double radius = 0;
	radius = 1.0 + cos(angle) / shrink;
	return radius;
}

/* -------------------------------------------------------
関数名:
	DrivingGear_rad
説明:
	ある方向(角度)の駆動歯車の半径を返す関数。
	(※非円形歯車なので、角度によって半径が違う)
引数:
	dist_of_center: 駆動歯車と従動歯車の中心間距離
	shrink:         各速度比の係数
	angle:          注目している角度
戻り値:
	angle[rad]方向の場合の駆動歯車の径
------------------------------------------------------- */
double DrivingGear_rad(const double dist_of_center, const double shrink, const double angle)
{
	double speed = Speed(shrink, angle);
	return dist_of_center * speed / (1.0 + speed);
}

/* -------------------------------------------------------
関数名:
	DrivenGear_rad
説明:
	ある方向(角度)の従動歯車の半径を返す関数。
	(※非円形歯車なので、角度によって半径が違う)
引数:
	dist_of_center: 駆動歯車と従動歯車の中心間距離
	shrink:         各速度比の係数
	angle:          注目している角度
戻り値:
	angle[rad]方向の場合の従動歯車の径
------------------------------------------------------- */
double DrivenGear_rad(const double dist_of_center, const double shrink, const double angle)
{
	double speed = Speed(shrink, angle);
	return dist_of_center / (1.0 + speed);
}

/* -------------------------------------------------------
関数名:
	calcPerimeter_driving
説明:
	駆動摩擦車周長算出関数。
	摩擦車の周長を数値積分で求める。
	微小区間は余弦定理から求める。
引数:
	dist_of_center:	歯車の中心間距離
	shrink:         各速度比の係数
	div_num:        描画点数
戻り値:
	なし
------------------------------------------------------- */
double calcPerimeter_driving(double dist_of_center, double shrink, int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}

/* -------------------------------------------------------
関数名:
	calcPerimeter_driving
説明:
	従動摩擦車周長算出関数。
	摩擦車の周長を数値積分で求める。
	微小区間は余弦定理から求める。
引数:
	dist_of_center:	歯車の中心間距離
	shrink:         各速度比の係数
	div_num:        描画点数
戻り値:
	なし
------------------------------------------------------- */
double calcPerimeter_driven(double dist_of_center, double shrink, int div_num)
{
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
		accum_perimeter += sqrt(	/* 余弦定理より、微小区間(距離）を算出 */
			radius1 * radius1 +
			radius2 * radius2 -
			2 * radius1 * radius2 * cos(2.0 * M_PI / div_num)
		);
	}
	return accum_perimeter;
}

/* -------------------------------------------------------
関数名:
	calcPerimeter_driving
説明:
	駆動歯車の歯位置・歯向きの作成関数。
引数:
	dist_of_center:	 歯車の中心間距離
	shrink:          各速度比の係数
	total_perimeter: 駆動摩擦車の周長
	tooth_num:		 歯数
	div_num:         描画点数
	tooth_angles:    摩擦車上の歯の向きの配列
	tooth_points:    摩擦車上の歯を生やす位置の配列
戻り値:
	なし
------------------------------------------------------- */
void calcToothAnglesAndPoints_driving(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = 0.0;	/* ←ここが駆動歯車と従動歯車が違う */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivingGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
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
/* -------------------------------------------------------
関数名:
	calcToothAnglesAndPoints_driven
説明:
	従動歯車の歯位置・歯向きの作成関数。
	駆動歯車の歯と噛み合うように、歯の位置はピッチの半分だけ進める。
引数:
	dist_of_center:	 歯車の中心間距離
	shrink:          各速度比の係数
	total_perimeter: 駆動摩擦車の周長
	tooth_num:		 歯数
	div_num:         描画点数
	tooth_angles:    摩擦車上の歯の向きの配列
	tooth_points:    摩擦車上の歯を生やす位置の配列
戻り値:
	なし
------------------------------------------------------- */
void calcToothAnglesAndPoints_driven(double dist_of_center, double shrink, double total_perimeter, int tooth_num, int div_num, vector<double>& tooth_angles, vector<coord2d>& tooth_points)
{
	const double pitch_offset = (total_perimeter / tooth_num) / 2.0;	/* 歯の位置をピッチの半分だけずらす */		/* ←ここが駆動歯車と従動歯車が違う */

	int teeth_index = 0;
	double accum_perimeter = 0.0;
	double radius1, radius2;
	for (int index = 0; index < div_num - 1; index++) {
		radius1 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index) / div_num);
		radius2 = DrivenGear_rad(dist_of_center, shrink, 2.0 * M_PI * (double)(index + 1.0) / div_num);
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

/* -------------------------------------------------------
関数名:
	storeGear_driving
説明:
	駆動歯車作成し配列に格納する関数。
	・ index番目の歯に注目し、前後の歯の位置と合わせた3点から仮想的な基準円中心を計算する
	・index番目の歯の周上の点を作成する。
	・index番目の歯を原点へ平行移動する
	・index番目の歯を歯の向きに合わせて回転移動する
	・index番目の歯を基準円上へ平行移動する
	・index番目の歯の周上の点を格納する
	・(index+1)番目の歯の処理へ移行
引数:
	module:         モジュール
	tooth_num:      歯数
	pressure_angle: 圧力角
	tooth_angles:   摩擦車上の歯の向きの配列
	tooth_points:   摩擦車上の歯を生やす位置の配列
	gear_plots:     歯車の周上の点の配列(出力用)
戻り値:
	なし
------------------------------------------------------- */
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
		thooth_drawing(diameter, module, pressure_angle, 0.5, tooth_points.at(index), plots);
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

/* -------------------------------------------------------
関数名:
	storeGear_driven
説明:
	従動歯車作成し配列に格納する関数。
	・ index番目の歯に注目し、前後の歯の位置と合わせた3点から仮想的な基準円中心を計算する
	・index番目の歯の周上の点を作成する。
	・index番目の歯を原点へ平行移動する
	・index番目の歯を歯の向きに合わせて回転移動する
	・index番目の歯を基準円上へ平行移動する
	・index番目の歯の周上の点を格納する
	・(index+1)番目の歯の処理へ移行
引数:
	module:         モジュール
	tooth_num:      歯数
	pressure_angle: 圧力角
	tooth_angles:   摩擦車上の歯の向きの配列
	tooth_points:   摩擦車上の歯を生やす位置の配列
	gear_plots:     歯車の周上の点の配列(出力用)
戻り値:
	なし
------------------------------------------------------- */
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
		thooth_drawing(diameter, module, pressure_angle, 0.5, tooth_points.at(index), plots);
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
		mirroring_y(plots);					/* ←ここが駆動歯車と従動歯車が違う */
		/* 作成した部分を格納 */
		gear_plots.insert(gear_plots.end(), plots.begin(), plots.end());
		plots.clear();
	}
}

/* -------------------------------------------------------
関数名:
	drawGear_driving
説明:
	駆動歯車描画関数。
引数:
	gear_plots:	歯車の周上の点の配列
	offset_x:   x軸方向のオフセット値
注意:
	FreeCADはdatファイルを読み込めるが、x軸は0以上でなければならない。
	よってoffset_xに適当な数値を入れておいてFreeCADで読み込めるようにする。
戻り値:
	なし
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
関数名:
	drawGear_driven
説明:
	駆動歯車描画関数。
引数:
	gear_plots:	歯車の周上の点の配列
	offset_x:   x軸方向のオフセット値
注意:
	FreeCADはdatファイルを読み込めるが、x軸は0以上でなければならない。
	よってoffset_xに適当な数値を入れておいてFreeCADで読み込めるようにする。
戻り値:
	なし
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
関数名:
	design_driving_gear
説明:
	駆動歯車作成処理のサブルーチン。
	・歯の位置と向きの配列を作成する
	・歯の作成と配置して歯車を作る
	・歯車をファイル(dat)に出力する
引数:
	dist_of_center:	歯車の中心間距離
	coefficient:    各速度比の係数
	module:	        モジュール
	pressure_angle: 圧力角
	tooth_num:		歯数
	div_num:        描画点数
戻り値:
	なし
------------------------------------------------------- */
void design_driving_gear(double dist_of_center, double coefficient, double module, double pressure_angle, int tooth_num, int div_num)
{
	vector<coord2d> centers;
	vector<double> angles;
	vector<coord2d> gears;
	const double total_perimeter = calcPerimeter_driving(dist_of_center, coefficient, div_num);
	cout << "<駆動歯車>" << endl;
	cout << "		total perimeter:		" << total_perimeter << endl;
	cout << "		tooth number (computed):	" << total_perimeter / M_PI / module << endl;
	cout << "		tooth number:			" << tooth_num << endl;
	cout << "		tooth number diffs:		" << abs(tooth_num - total_perimeter / M_PI / module) << endl;

	calcToothAnglesAndPoints_driving(dist_of_center, coefficient, total_perimeter, tooth_num, div_num, angles, centers);
	storeGear_driving(module, tooth_num, pressure_angle, angles, centers, gears);
	drawGear_driving(gears, 100);
}
/* -------------------------------------------------------
関数名:
	design_driven_gear
説明:
	従動歯車作成処理のサブルーチン。
	・歯の位置と向きの配列を作成する
	・歯の作成と配置して歯車を作る
	・歯車をファイル(dat)に出力する。
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
	cout << "<従動歯車>" << endl;
	cout << "		total perimeter:		" << total_perimeter << endl;
	cout << "		tooth number (computed):	" << total_perimeter / M_PI / module << endl;
	cout << "		tooth number:			" << tooth_num << endl;
	cout << "		tooth number diffs:		" << abs(tooth_num - total_perimeter / M_PI / module) << endl;

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
void thooth_drawing(double img_std_diameter, double module, double pressure_angle, double clearance, coord2d vct, vector<coord2d>& plots)
{
	const double plot_num = 30;	/* 描画点数 */
	const coord2d origin_point(0.0, 0.0);	/* 原点 */
	double involute_start_angle;
	const double involute_end_angle =		/* x軸を基準として下に何度[rad]でインボリュート曲線が歯先円に達するか */
		acos(
			(img_std_diameter / 2.0 * cos(pressure_angle)) /
			(img_std_diameter / 2.0 + module));

	coord2d tmp_point;
	if (img_std_diameter * cos(pressure_angle) / 2.0 > img_std_diameter / 2.0 - (1.0 + clearance) * module) {
		involute_start_angle = 	/* x軸を基準として下に何度[rad]からインボリュート歯を描き始めるか */
			TT_to_Angle(img_std_diameter, module, pressure_angle, 0.9, img_std_diameter * cos(pressure_angle));
		/* 歯底円上の点を1点描く */
		coord2d point = coord2d(
			(img_std_diameter / 2.0 - (1.0 + clearance) * module) * cos(-1.0 * involute_start_angle),
			(img_std_diameter / 2.0 - (1.0 + clearance) * module) * sin(-1.0 * involute_start_angle)
		);
		plots.push_back(point);
	}
	else {
		involute_start_angle = 	/* x軸を基準として下に何度[rad]からインボリュート歯を描き始めるか */
			TT_to_Angle(img_std_diameter, module, pressure_angle, 0.9, img_std_diameter - (1.0 + clearance) * module);
	}

	/* インボリュート曲線部分を描く */
	vector<coord2d> tmp_points;
	double dist = distance(origin_point, origin_point);
	for (double angle = 0.0;
		angle <= involute_end_angle;				/* 正しいはずなんだけど、この条件だと歯先と滑らかに繋がらないんだよね。計算の誤差かな... */
		//dist < img_std_diameter / 2.0 + module;	/* 「歯先半径に達するまで」を条件にしてもやっぱり一致しない。もうどうしようもないね... */
		angle += involute_end_angle / plot_num)
	{
		tmp_point = involute_drawing(angle, pressure_angle, img_std_diameter);	/* 角度angleのインボリュート曲線上の点を描画 */
		rotmove(-1.0 * involute_start_angle, tmp_point);						/* 角度involute_start_angle分だけ「下に」下げる */
		plots.push_back(tmp_point);
		dist = distance(tmp_point, origin_point);
	}
	double tmp = distance(tmp_point, origin_point);
	/* 下半分を反転して上半分を描く */
	for (int index = plots.size() - 1; index >= 0; index--)
	{
		coord2d additional(plots.at(index).getx(), -1.0 * plots.at(index).gety());
		plots.push_back(additional);
	}

}


/* -------------------------------------------------------
関数名:
	involute_drawing
説明:
	インボリュート曲線上の点の作成関数。
引数:
	angle: 「糸の先端」「基礎円中心」「基礎円と糸の接点」のなす角
	pressure_angle: 圧力角
	std_diameter: 基準円半径
戻り値:
	インボリュート曲線上の座標
------------------------------------------------------- */
coord2d involute_drawing(double angle, double pressure_angle, double std_diameter)
{
	return coord2d(
		std_diameter * cos(pressure_angle) * cos(inv(angle)) / cos(angle) / 2.0,
		std_diameter * cos(pressure_angle) * sin(inv(angle)) / cos(angle) / 2.0
	);
}


/* -------------------------------------------------------
関数名:
	paramove
説明:
	座標の平行移動関数。
	引数で渡された座標配列から要素を取得する。
	(dx, dy)平行移動させた座標を作成し同じ要素に格納しなおす。
引数:
	dx: x軸方向の移動距離
	dy: y軸方向の移動距離
	plots: x-y座標の位置を格納する配列
戻り値:
	なし
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
関数名:
	rotmove
説明:
	座標の回転移動関数。
	引数で渡された座標配列から要素を取得する。
	theta回転させた座標を作成し同じ要素に格納しなおす。
引数:
	theta: 回転角度[rad]
	plots: x-y座標の位置を格納する配列
戻り値:
	なし
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
関数名:
	rotmove
説明:
	座標の回転移動関数。
	引数で渡された座標をtheta回転させた座標を引数に格納しなおす。
引数:
	theta: 回転角度[rad]
	plot: x-y座標
戻り値:
	なし
------------------------------------------------------- */
void rotmove(double theta, coord2d& plot)
{
	double tmp_x = plot.getx();
	double tmp_y = plot.gety();
	plot.setx(cos(theta) * tmp_x - sin(theta) * tmp_y);
	plot.sety(sin(theta) * tmp_x + cos(theta) * tmp_y);
}

/* -------------------------------------------------------
関数名:
	mirroring_y
説明:
	座標の反転(ミラー)関数。
	引数で渡された座標配列から要素を取得する。
	y軸に対して対称な座標を作成し同じ要素に格納しなおす。
引数:
	plots: x-y座標の位置を格納する配列。
戻り値:
	なし
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
double TT_to_Angle(double std_diameter, double module, double pressure_angle, double backlash, double target_diameter)
{
	double target_angle = 0;
	const double phi = acos(std_diameter * cos(pressure_angle) / target_diameter);	/* 糸の長さが なす角 */
	const double S_std = M_PI * module / 2.0;										/* 基準円状の歯厚(標準だとπm/2なんだってさ) */
	const double S_target =															/* target歯厚 */
		target_diameter *
		(S_std / std_diameter + inv(pressure_angle) - inv(phi));
	target_angle = (S_target / 2.0) / (target_diameter / 2.0);						/* (歯厚の半分) / (target半径) / 2 */
	target_angle *= backlash;														/* バックラッシとして歯厚を95%にしておくね */

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
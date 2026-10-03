#include "stdafx.h"
#include "utility.h"

Point2f rotateFlyCenter(Point2f p, int image_width, int image_height)
{
	Point2f temp, refPt;

	//move point to origin and rotate by 15 degrees due to the tilt of the galvo x-mirror
	temp.x = (cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) - sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));
	temp.y = (sin(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.x - image_width / 2) + cos(-GALVO_X_MIRROR_ANGLE * CV_PI / 180)*(p.y - image_height / 2));

	refPt.x = (image_width / 2) + temp.x;
	refPt.y = (image_height / 2) + temp.y;

	//printf("[%f %f]\n", pt.at<double>(0, 0), pt.at<double>(1, 0));
	//printf("[%f %f]\n", refPt.at<double>(0, 0), refPt.at<double>(1, 0));

	return refPt;
}

float dist(Point2f p1, Point2f p2)
{
	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	return(sqrt(dx*dx + dy*dy));
}

int findClosestPoint(
    Point2f pt,
    const vector<Point2f>& nbor
)
{
    if (nbor.empty())
        return -1;

    int closest_index = 0;

    float dx = nbor[0].x - pt.x;
    float dy = nbor[0].y - pt.y;

    float best_dist_sq =
        dx * dx +
        dy * dy;

    for (size_t i = 1; i < nbor.size(); i++)
    {
        dx = nbor[i].x - pt.x;
        dy = nbor[i].y - pt.y;

        float dist_sq =
            dx * dx +
            dy * dy;

        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            closest_index = static_cast<int>(i);
        }
    }

    return closest_index;
}

static float cross2D(
    const cv::Point2f& a,
    const cv::Point2f& b
)
{
    return a.x * b.y - a.y * b.x;
}


bool findContourRayIntersection(
    const vector<Point>& contour,
    Point2f origin,
    Point2f direction,
    Point2f& intersection
)
{
    if (contour.size() < 2)
        return false;

    float direction_norm =
        sqrt(
            direction.x * direction.x +
            direction.y * direction.y
        );

    if (direction_norm < 1e-6f)
        return false;

    // Normalize the ray direction.
    Point2f ray_dir(
        direction.x / direction_norm,
        direction.y / direction_norm
    );

    bool found = false;
    float nearest_t = 1e9f;

    for (size_t i = 0; i < contour.size(); i++)
    {
        size_t next =
            (i + 1) % contour.size();

        Point2f p1(
            static_cast<float>(contour[i].x),
            static_cast<float>(contour[i].y)
        );

        Point2f p2(
            static_cast<float>(contour[next].x),
            static_cast<float>(contour[next].y)
        );

        Point2f segment(
            p2.x - p1.x,
            p2.y - p1.y
        );

        Point2f relative(
            p1.x - origin.x,
            p1.y - origin.y
        );

        float denominator =
            cross2D(ray_dir, segment);

        // Ray and contour segment are approximately parallel.
        if (fabs(denominator) < 1e-6f)
            continue;

        float t =
            cross2D(relative, segment) /
            denominator;

        float u =
            cross2D(relative, ray_dir) /
            denominator;

        // t > 0: intersection is in front of the centroid.
        // 0 <= u <= 1: intersection is on this contour segment.
        if (
            t > 0.0f &&
            u >= 0.0f &&
            u <= 1.0f &&
            t < nearest_t
            )
        {
            nearest_t = t;
            found = true;
        }
    }

    if (!found)
        return false;

    intersection.x =
        origin.x +
        nearest_t * ray_dir.x;

    intersection.y =
        origin.y +
        nearest_t * ray_dir.y;

    return true;
}

int ConvertTimeToFPS(int ctime, int ltime)
{
	int dtime;

	if (ctime < ltime)
		dtime = ctime + (8000 - ltime);
	else
		dtime = ctime - ltime;

	if (dtime > 0)
		dtime = 8000 / dtime;
	else
		dtime = 0;

	return dtime;
}

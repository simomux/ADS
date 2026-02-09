#include "tracker/Tracker.h"

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 4.5; // meters
    covariance_threshold = 1.5;
    loss_threshold = 250; //number of frames the track has not been seen
}
Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    /* std::vector<Tracklet> tracks_to_keep;

    for (size_t i = 0; i < tracks_.size(); ++i)
    {
        // Implement logic to discard old tracklets
        // logic_to_keep is a dummy placeholder to make the code compile and should be subsituted with the real condition
        bool logic_to_keep = true;
        if (logic_to_keep)
            tracks_to_keep.push_back(tracks_[i]);
    }

    tracks_.swap(tracks_to_keep); */

    auto it = tracks_.begin();
    while (it != tracks_.end())
    {
        // Remove tracklet if it hasn't been seen for too many frames
        // OR if uncertainty (covariance) is too high
        if (it->getLossCount() > loss_threshold || it->getXCovariance() > covariance_threshold || it->getYCovariance() > covariance_threshold)
            it = tracks_.erase(it);
        else
            ++it;
    }
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{

    //Remind this vector contains a pair of tracks and its corresponding
    associated_track_det_ids_.clear();

    for (size_t i = 0; i < tracks_.size(); ++i)
    {

        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < associated_detections.size(); ++j)
        {
            // Implement logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_) 
            
            // Skip if detection already associated
            if (associated_detections[j])
                continue;

            // Delta
            double delta_x = centroids_x[j] - tracks_[i].getX();
            double delta_y = centroids_y[j] - tracks_[i].getY();

            double euclidean_sq = delta_x*delta_x + delta_y*delta_y;
            if (euclidean_sq > distance_threshold_ * 2.0)  // Soglia generosa
                continue;  // Scarta rapidamente

            // 2x2 covariance matrix
            Eigen::Matrix2d covariance;
            covariance << tracks_[i].getXCovariance(), 0,
                          0, tracks_[i].getYCovariance();

            Eigen::Vector2d delta(delta_x, delta_y);

            // Mahalanobis distance
            double mahal_dist_squared = delta.transpose() * covariance.inverse() * delta;

            if (mahal_dist_squared < min_dist)
            {
                min_dist = mahal_dist_squared;
                closest_point_id = j;
            }
            
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(closest_point_id, i));
            associated_detections[closest_point_id] = true;
        }
    }
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{

    std::vector<bool> associated_detections(centroids_x.size(), false);

    // Predict the position
    //For each track --> Predict the position of the tracklets
    for (size_t i = 0; i < tracks_.size(); ++i)
        tracks_[i].predict();
    
    // Associate the predictions with the detections
    this->dataAssociation(associated_detections, centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto det_id = associated_track_det_ids_[i].first;
        auto track_id = associated_track_det_ids_[i].second;
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    // Remove dead tracklets
    this->removeTracks();
    

    // Add new tracklets
    this->addTracks(associated_detections, centroids_x, centroids_y);

}

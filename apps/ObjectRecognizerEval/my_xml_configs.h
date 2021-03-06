#pragma once

#include "boost_xml_editor.h"

std::vector< std::pair< std::vector<XMLChange>, bool > > loadChanges()
{
    std::vector< std::pair< std::vector<XMLChange>, bool > > grouped_changes;

  if(0)
  {
    {
            bool local_search_is_okay = false;
            std::pair< std::vector<XMLChange>, bool > grouped_change;
            grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "resolution_mm_", {"3"}, false ) );
            grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "resolution_mm_", {"5"}, false ) );
            grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "resolution_mm_", {"7"}, false ) );
            grouped_change.second = local_search_is_okay;
            grouped_changes.push_back( grouped_change );
    }

    {
        bool local_search_is_okay = true;
        std::pair< std::vector<XMLChange>, bool > grouped_change;
        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "knn_", {"2"} , false) );
        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "knn_", {"5"} , false) );
        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "knn_", {"7"} , false) );
        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "knn_", {"10"} , false) );
        grouped_change.second = local_search_is_okay;
        grouped_changes.push_back( grouped_change );
    }

    {
        bool local_search_is_okay = true;
        std::pair< std::vector<XMLChange>, bool > grouped_change;
        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_thresh_", {"3"}, false ) );
        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_thresh_", {"5"}, false ) );
        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_thresh_", {"7"}, false ) );
        grouped_change.second = local_search_is_okay;
        grouped_changes.push_back( grouped_change );
    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_size_", {"0.005"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_size_", {"0.01"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "cg_size_", {"0.05"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "filter_border_pts_", {"0"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "filter_border_pts_", {"1"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "distance_metric_", {"1"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "distance_metric_", {"4"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "kdtree_num_trees_", {"4"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "kdtree_num_trees_", {"8"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "kdtree_splits_", {"512"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "kdtree_splits_", {"264"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/sift_config.xml", "kdtree_splits_", {"1024"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

//    {
//        bool local_search_is_okay = true;
//        std::pair< std::vector<XMLChange>, bool > grouped_change;
//        grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "smoothing_radius_", {"1"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "smoothing_radius_", {"2"}, false ) );
//        grouped_change.first.push_back( XMLChange("cfg/hv_config.xml", "smoothing_radius_", {"3"}, false ) );
//        grouped_change.second = local_search_is_okay;
//        grouped_changes.push_back( grouped_change );
//    }

      {
  //            bool local_search_is_okay = false;
  //            std::pair< std::vector<XMLChange>, bool > grouped_change;
  //            grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "do_sift_", {"1"}, false ) );
  //            grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "do_esf_", {"1"}, false ) );
  //            grouped_change.second = local_search_is_okay;
  //            grouped_changes.push_back( grouped_change );
      }

      {
  //            bool local_search_is_okay = false;
  //            std::pair< std::vector<XMLChange>, bool > grouped_change;
  //            grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "keypoint_support_radii_", {"0.04", "0.05", "0.06"}, true ) );
  //            grouped_change.first.push_back( XMLChange("cfg/multipipeline_config.xml", "keypoint_support_radii_", {"0.12", "0.08"}, true ) );
  //            grouped_change.second = local_search_is_okay;
  //            grouped_changes.push_back( grouped_change );
      }
   }

//    std::vector<std::vector<XMLChange> > all_changes;

//    //first empty (default settings)
//    std::vector<XMLChange> foo_change;
//    all_changes.push_back(foo_change);

//    //now only single changes
//    for(size_t i=0; i <grouped_changes.size(); i++)
//    {
//        for( const XMLChange &chg : grouped_changes[i])
//        {
//            std::vector<XMLChange> single_change;
//            single_change.push_back( chg );
//            all_changes.push_back(single_change);
//        }
//    }

//    // now pairwise changes
//    for(size_t i=0; i <grouped_changes.size(); i++)
//    {
//        for(size_t j=i+1; j <grouped_changes.size(); j++)
//        {
//            if(i==j)
//                continue;

//            const std::vector<XMLChange> &grouped_change_a = grouped_changes[i];
//            const std::vector<XMLChange> &grouped_change_b = grouped_changes[j];

//            for( const XMLChange &chg_a : grouped_change_a)
//            {
//                for( const XMLChange &chg_b : grouped_change_b)
//                {
//                    std::vector<XMLChange> pairwise_change;
//                    pairwise_change.push_back( chg_a );
//                    pairwise_change.push_back( chg_b );
//                    all_changes.push_back( pairwise_change);
//                }
//            }
//        }
//    }

    return grouped_changes;
}

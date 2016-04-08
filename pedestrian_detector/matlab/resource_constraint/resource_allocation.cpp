#include "resource_allocation.h"

DARP::DARP(const int & width_,
        const int & height_,
        const double & max_relative_capacity_,
        const int & max_items_,
        const int & min_width_,
        const int & min_height_) :
            width(width_), 
                    height(height_),
                    max_relative_capacity(max_relative_capacity_), //percentage
                    max_items(max_items_),
                    total_capacity(max_relative_capacity_*width_*height_),
                    optimizer(new Knapsack(total_capacity,max_items,width,height)),
                    min_width(min_width_),
                    min_height(min_height_)
            {
                // ALLOCATE MEMORY FOR AUXILIARY PROBABILITY MAPS
                item_weight.resize(max_items);
                item_value.resize(max_items);
                std::fill(item_value.begin(),item_value.end(),0.0);
                std::fill(item_weight.begin(),item_weight.end(),10000000);
                
                // ALLOCATE MEMORY FOR OTHER AUXILIARY VARS
                x=cv::Mat(2,1,CV_64F);
                std_dev=cv::Mat(2,1,CV_64F);
                centroid_std_dev=cv::Mat(2,1,CV_64F);
                size_std_dev=cv::Mat(2,1,CV_64F);
                size_means=cv::Mat(2,1,CV_64F);
                thresholds=cv::Mat(2,1,CV_64F);
                cov=cv::Mat(2,2,CV_64F,cv::Scalar(0.0));
                probability_maps.resize(1);
                
                // INITIALIZE WITH A GAUSSIAN CENTERED
                probability_maps[0]=cv::Mat(height_,width_,CV_64F,cv::Scalar(0));
                
                cv::Mat mu(2,1,CV_64F);
                mu.at<double>(0,0)=width/2.0;
                mu.at<double>(1,0)=height/2.0;
                
                cv::Mat cov(2,2,CV_64F,cv::Scalar(0.0));
                cov.at<double>(0,0)=1000.0;
                cov.at<double>(1,1)=1000.0;
                
                tic();
                
                cv::Mat x(2,1,CV_64F);
                for(int i=0;i<width;++i)
                {
                    for(int j=0; j<height;++j)
                    {
                        x.at<double>(0,0)=i;
                        x.at<double>(1,0)=j;
                        probability_maps[0].at<double>(j,i)=gaussian(x,mu,cov);
                    }
                }
                std::cout << toc_()<< std::endl;
            };
            
            void DARP::computeValues(
                    const cv::Mat & state_means,
                    const cv::Mat & state_variances)
            {
                
                int nItems=state_means.rows;
                
                if(nItems==0)
                    return;
                rois.resize(nItems);
                
                
                // For each tracked object...
                for(int o=0; o<nItems;++o)
                {
                    size_std_dev.at<double>(0,0)=sqrt(state_variances.at<double>(o,2));
                    size_std_dev.at<double>(1,0)=sqrt(state_variances.at<double>(o,2));
                    centroid_std_dev.at<double>(0,0)=sqrt(state_variances.at<double>(o,0));
                    centroid_std_dev.at<double>(1,0)=sqrt(state_variances.at<double>(o,1));
                    std_dev=centroid_std_dev+size_std_dev;
                    
                    size_means.at<double>(0,0)=min_width*sqrt(state_means.at<double>(o,2));
                    size_means.at<double>(1,0)=min_height*sqrt(state_means.at<double>(o,2));
                    thresholds=(size_means+1.0*std_dev)/2.0;// Half size each side)PARAMETRIZE THIS!!!
                    // Max capacity (find better solution)
                    //if(o>max_items)
                    //   continue;
                    
                    
                    //if(thresholds.at<double>(o,0)>250||thresholds.at<double>(o,1)>250)
                    //    continue;
                    
                    // don't go further than this
                    int width_threshold=thresholds.at<double>(0,0);
                    int height_threshold=thresholds.at<double>(0,1);
                    
                    cov.at<double>(0,0)=std_dev.at<double>(0,0)*std_dev.at<double>(0,0);
                    cov.at<double>(1,1)=std_dev.at<double>(0,1)*std_dev.at<double>(0,1);
                    
                    int width_begin=-width_threshold+round(state_means.at<double>(o,0));
                    if(width_begin<0)
                    {
                        width_begin=0;
                        width_threshold=round(state_means.at<double>(o,0));
                    }
                    
                    int width_end=width_threshold+round(state_means.at<double>(o,0));
                    if(width_end>width)
                    {
                        width_end=width;
                        width_threshold=width-round(state_means.at<double>(o,0));
                    }
                    
                    int height_begin=-height_threshold+round(state_means.at<double>(o,1));
                    if(height_begin<0)
                    {
                        height_begin=0;
                        height_threshold=round(state_means.at<double>(o,1));
                    }
                    
                    int height_end=height_threshold+round(state_means.at<double>(o,1));
                    if(height_end>height)
                    {
                        height_end=height;
                        height_threshold=height-round(state_means.at<double>(o,1));
                    }
                    
                    // Get ROI
                    double roi_width=width_end-width_begin;
                    double roi_height=height_end-height_begin;
                    rois[o]=cv::Rect(width_begin,
                            height_begin,
                            roi_width,
                            roi_height);
                    
                }
                
                double minFraction=0.1;
                
                std::vector<bool> associated_rois;
                nItems=rois.size();
                item_value.resize(nItems);
                probability_maps.resize(nItems);
                
                //std::fill(item_value.begin(),item_value.end(),0.0);
                //std::fill(item_weight.begin(),item_weight.end(),std::numeric_limits<int>::max());
                
                // FIND NEW ROIS AND FILL VALUES
                
                bool merges;
                do
                {
                    int i=-1;
                    merges=false;
                    associated_rois.resize(rois.size());
                    std::fill(associated_rois.begin(),associated_rois.end(),false);

                    while ( i < (int)(rois.size()-1) )
                    {
                        
                        ++i;
                        if(associated_rois[i])
                        {
                            continue;
                        }
                        
                        int j=i+1;
                        int j_real=j;
                        double value=gaussian(state_means(cv::Range(i,i+1),cv::Range(0,2)).t(),state_means(cv::Range(i,i+1),cv::Range(0,2)).t(),cov);
                        item_value[i]=value*rois[i].area();
                        while(j<rois.size())
                        {
                            if ( (rois[i]&rois[j]).area() > minFraction * std::min(rois[i].area(), rois[j].area()) )
                            {
                                value=gaussian(state_means(cv::Range(j_real,j_real+1),cv::Range(0,2)).t(),state_means(cv::Range(j_real,j_real+1),cv::Range(0,2)).t(),cov);
                                item_value[i]+=value*rois[j].area();
                                rois[i]=cv::Rect(rois[i]|rois[j]);
                                associated_rois[j_real]=true;
                                
                                rois.erase(rois.begin()+j);
                                item_value.erase(item_value.begin()+j);
                                merges=true;
                            }
                            else
                            {
                                ++j;
                            }
                            ++j_real;
                        }
                        
                        item_value[i]/=rois[i].area();
                    }
                }
                while(merges);
                
                
                item_weight.resize(rois.size());
                for(int i=0; i<item_weight.size();++i)
                {
                    item_weight[i]=rois[i].area();
                }
                // Fill
                /*for(int o=0; o<nItems;++o)
                 * {
                 * double best=gaussian(state_means(cv::Range(o,o+1),cv::Range(0,2)).t(),state_means(cv::Range(o,o+1),cv::Range(0,2)).t(),cov);
                 *
                 * item_value[o]=rois[o].area()*best;
                 * probability_maps[o]=cv::Mat(height,width,CV_64F,cv::Scalar(0));
                 * probability_maps[o](rois[o])=best;
                 *
                 * /*item_value[o]=0.0;
                 * for(int u=rois[o].x;u<rois[o].x+rois[o].width;++u)
                 * {
                 * for(int j=rois[o].y; j<rois[o].y+rois[o].height;++j)
                 * {
                 * x.at<double>(0,0)=u;
                 * x.at<double>(1,0)=j;
                 *
                 * probability_maps[o].at<double>(j,u)=best;
                 * item_value[o]+=best;
                 * }
                 * }
                 * }*/
            }
            
            std::vector<cv::Rect> DARP::getROIS(const cv::Mat & state_means,
                    const cv::Mat & state_variances)
            {
                tic();
                computeValues(state_means, state_variances);
                std::cout << "time elapsed (computing values before optimization):" << toc_()<< std::endl;
                std::vector<cv::Rect> best_rois;
                
                if(item_value.size()==0) // Couldn't compute values for some reason
                    return best_rois;
                tic();
                std::vector<int> items=optimize(item_value,item_weight);
                std::cout << "time elapsed (knapsack optimization):" << toc_()<< std::endl;
                
                
                best_rois.resize(items.size());
                
                for(int i=0; i< items.size();++i)
                {
                    best_rois[i]=(rois[items[i]]);
                }
                
                return best_rois;
            }
            
            double DARP::gaussian(const cv::Mat & x,
                    const cv::Mat & mu,
                    const cv::Mat & cov)
            {
                cv::Mat diff=x-mu;
                double det=determinant(cov);
                double aux=(0.5/M_PI)*(1.0/sqrt(det));
                
                cv::Mat exponent=-0.5*diff.t()*cov.inv()*diff;
                
                return aux*exp(exponent.at<double>(0));
            }
            
            std::vector<int> DARP::optimize(const std::vector<double> & value,
                    const std::vector<int> & weight)
            {
                optimizer->clear_items();
                //optimizer=boost::shared_ptr<Knapsack>(new Knapsack(total_capacity,value.size(),width,height));
                
                for(int i=0; i<value.size();++i)
                {
                    //std::cout << "value["<<i<<"]:"<< value[i] << std::endl;
                    //std::cout << "weight["<<i<<"]:"<< weight[i] <<" of " << total_capacity << " "<< width*height << std::endl;
                    
                    optimizer->add_items(value[i],weight[i]);
                }
                double solution  = optimizer->solve();
                //std::cout << *optimizer;
                std::vector<item> resultItems;
                std::vector<int> resultItemsIndices;
                optimizer->get_items_selected(resultItems,resultItemsIndices);
                /*int i=0;
                 * for ( std::vector<item>::iterator itr = resultItems.begin();
                 * itr != resultItems.end();
                 * ++itr)
                 * {
                 * std::cout    << '\t' << '\t' << "(V:"
                 * << std::setw(2)
                 * << itr->value
                 * << ", "
                 * << "W:"
                 * << itr->capacity
                 * << ")   index:"
                 * << resultItemsIndices[i++]
                 * << std::endl;
                 * }*/
                return resultItemsIndices;
            }
            
            

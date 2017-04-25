/**
 * $Id$
 * 
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (C) 2015:
 *
 *    Johann Prankl, prankl@acin.tuwien.ac.at
 *    Aitor Aldoma, aldoma@acin.tuwien.ac.at
 *
 *      Automation and Control Institute
 *      Vienna University of Technology
 *      Gusshausstraße 25-29
 *      1170 Vienn, Austria
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Johann Prankl, Aitor Aldoma
 *
 */

#include <iostream>
#include <v4r/texturing/hittingset.h>

namespace v4r
{
namespace texturing
{

std::set<int> HittingSet::solve(std::vector<int> set, std::vector<std::vector<int> > subsets)
{
    std::cout << "solve hitting set problem" << std::endl;
    std::cout << "nr of subsets: " << subsets.size() << std::endl;
    std::cout << "Remaining images: " << set.size() << std::endl;

    if (set.size() > 20)
    {
        return approximateSolution(set, subsets);
    }


    std::set<int> combinations;
    for (int i=1;i<=(int)set.size();i++)
    {
        std::cout << "Start iteration " << i << std::endl;

        combinations.clear();
        bool match = combination(&set, &subsets, &combinations, 0, i);

        if (match)
        {
            std::cout << "match for " << i << " images" << std::endl;
            break;
        }
    }

    return combinations;
}

std::set<int> HittingSet::approximateSolution(std::vector<int> set, std::vector<std::vector<int> > subsets)
{
    std::set<int> result;

    while (subsets.size() > 0)
    {
        int bestIndex = -1;
        int bestValue = 0;

        for (int i=0;i<(int)set.size();i++)
        {
            int count = 0;

            for (int j=0;j<(int)subsets.size();j++)
            {
                for (int k=0;k<(int)subsets[j].size();k++)
                {
                    if (subsets[j][k] == set[i])
                    {
                        count++;
                    }
                }
            }

            if (count > bestValue)
            {
               bestIndex = i;
               bestValue = count;
            }
        }

        if (bestIndex == -1)
        {
            std::cout << "WARNING: SHOULD NOT HAPPEN !!!!!!!!!" << std::endl;
            std::cout << "first subset ";

            for (int s=0;s<(int)subsets[0].size();s++)
            {
                std::cout << subsets[0][s] << " ";
            }

            std::cout << std::endl;

            std::cout << "remaining set ";

            for (int s=0;s<(int)set.size();s++)
            {
                std::cout << set[s] << " ";
            }

            std::cout << std::endl;

            return result;
        }

        for (int j=0;j<(int)subsets.size();j++)
        {
            bool match = false;

            for (int k=0;k<(int)subsets[j].size();k++)
            {
                if (subsets[j][k] == set[bestIndex])
                {
                    match = true;
                }
            }

            if (match)
            {
                subsets.erase(subsets.begin() + j);
                j--;
            }
        }

        set.erase(set.begin() + bestIndex);
        result.insert(bestIndex);


        std::cout << "Add image to set: " << bestIndex << std::endl;
        std::cout << "Remaining images : " << set.size() << std::endl;
        std::cout << "Remaining subsets: " << subsets.size() << std::endl;
    }



    std::cout << "MATCH FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


    std::cout << "Combinations " << std::endl;

    std::set<int>::iterator it;
    for (it = result.begin(); it != result.end(); ++it)
    {
        std::cout << *it << " ";
    }

    std::cout << std::endl;

    return result;
}

bool HittingSet::combination(std::vector<int> *set, std::vector<std::vector<int> > *subsets, std::set<int> *combinations, int offset, int k)
{

    if (k == 0) {
        // check if finished

        for (int polyIndex=0;polyIndex<(int)subsets->size();polyIndex++)
        {
            bool match = false;

            std::set<int>::iterator it;
            for (it = combinations->begin(); it != combinations->end(); ++it)
            {
                for (int c=0;c<(int)subsets->at(polyIndex).size();c++)
                {
                    if (subsets->at(polyIndex).at(c) == *it)
                    {
                        match = true;
                        break;
                    }
                }

                if (match)
                {
                    break;
                }
            }

            if (!match)
            {
                return false;
            }
        }

        std::cout << "MATCH FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


        std::cout << "Combinations " << std::endl;

        std::set<int>::iterator it;
        for (it = combinations->begin(); it != combinations->end(); ++it)
        {
            std::cout << *it << " ";
        }

        std::cout << std::endl;

        /*
        std::cout << "Single matches " << std::endl;

        for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
        {
            std::cout << *it << " ";
        }

        std::cout << std::endl;
        */

        return true;
    }

    for (int i = offset; i <= (int)set->size() - k; ++i) {
        std::vector<int>::iterator it = set->begin();
        std::advance(it, i);
        int x = *it;

        combinations->insert(x);
        bool match = combination(set, subsets, combinations, i+1, k-1);

        if (match)
        {
            return true;
        }

        combinations->erase(x);
    }

    return false;
}

}
}

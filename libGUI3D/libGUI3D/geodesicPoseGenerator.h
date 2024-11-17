//
// Created by sc on 3/23/21.
//

#ifndef MESHRENDER_GEODESICPOSEGENERATOR_H
#define MESHRENDER_GEODESICPOSEGENERATOR_H

namespace SC {
    template<typename T>
    class GeodesicGenerator{
    public:
        typedef T scalarT;
        typedef std::array<scalarT,3> pointT;
        std::vector<pointT> points;

        GeodesicGenerator(unsigned int depth){
            auto X = 0.525731112119133606;
            auto Z = 0.850650808352039932;
            std::vector<pointT> positions = {
                    {-X, 0, Z}, {X,0,Z}, {-X,0,-Z}, {X,0,-Z},
                    {0,Z,X},{0,Z,-X},{0,-Z,X},{0,-Z,-X},
                    {Z,X,0},{-Z,X,0},{Z,-X,0},{-Z,-X,0}
            };
            std::vector<std::array<unsigned int,3>> indices = {
                    {0,4,1},{0,9,4},{9,5,4},{4,5,8},{4,8,1},
                    {8,10,1},{8,3,10},{5,3,8},{5,2,3},{2,7,3},
                    {7,10,3},{7,6,10},{7,11,6},{11,0,6},{0,1,6},
                    {6,1,10},{9,0,11},{9,11,2},{9,2,5},{7,2,11}
            };
            for(const auto &index : indices){
                sub_divide(positions[index[0]],positions[index[1]],positions[index[2]],depth);
            }
        }
    private:
        std::set<pointT> unique_points;
        void unique_add(const pointT &pos){
            if(unique_points.find(pos) == unique_points.end()) {
                unique_points.insert(pos);
                points.push_back(pos);
            }
        }
        void sub_divide(const pointT& v1, const pointT& v2, const pointT& v3, unsigned int depth){
            if(depth == 0) {
                unique_add(v1);
                unique_add(v2);
                unique_add(v3);
                return;
            }
            depth-=1;
            auto v12 = normalize(add(v1,v2));
            auto v23 = normalize(add(v2,v3));
            auto v31 = normalize(add(v3,v1));
            sub_divide(v1, v12, v31, depth);
            sub_divide(v2, v23, v12, depth);
            sub_divide(v3, v31, v23, depth);
            sub_divide(v12, v23, v31, depth);
        }
        pointT normalize(pointT p){
            scalarT dist = 0;
            for (auto pp : p)
                dist += pp*pp;
            dist = sqrt(dist);

            pointT pout;
            for(size_t i=0;i<p.size();++i)
                pout[i] = p[i]/dist;
            return pout;
        }
        pointT add(const pointT &p1, const pointT &p2){
            pointT  pout;
            for(size_t i=0;i<p1.size();++i){
                pout[i] = p1[i]+p2[i];
            }
            return pout;
        }
    };
}

#endif //MESHRENDER_GEODESICPOSEGENERATOR_H

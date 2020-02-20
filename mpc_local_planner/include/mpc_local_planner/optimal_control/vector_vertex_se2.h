/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
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
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#ifndef VECTOR_SE_VERTEX_H_
#define VECTOR_SE_VERTEX_H_

#include <corbo-optimization/hyper_graph/vector_vertex.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <corbo-core/types.h>

#include <memory>
#include <vector>

namespace mpc_local_planner {

/**
 * @brief Vertex specialization for vectors in SE2
 *
 * This class defines a vector vertex in which the first
 * three components must be defined in SE2.
 * Whereas the first two states are ordinary real numbers on (-inf, inf),
 * the third component must remain in [-pi, pi).
 *
 * The minimum dimension of this vertex is 3.
 * This vertex allows for larger dimensions of 3,
 * but these components are considered as standard real numbers on (-inf, inf).
 *
 * @see corbo::VertexInterface corbo::PartiallyFixedVectorVertex
 *      corbo::HyperGraph corbo::EdgeInterface corbo::ScalarVertex
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class VectorVertexSE2 : public corbo::VectorVertex
{
 public:
    using Ptr  = std::shared_ptr<VectorVertexSE2>;
    using UPtr = std::unique_ptr<VectorVertexSE2>;

    //! Default constructor
    VectorVertexSE2() = default;

    explicit VectorVertexSE2(bool fixed) : corbo::VectorVertex(fixed) {}

    //! Construct and allocate memory for a given dimension
    explicit VectorVertexSE2(int dimension, bool fixed = false) : VectorVertex(dimension, fixed) {}

    //! Construct vertex with given values
    explicit VectorVertexSE2(const Eigen::Ref<const Eigen::VectorXd>& values, bool fixed = false) : VectorVertex(values, fixed) {}

    //! Construct vertex with given values, lower and upper bounds
    explicit VectorVertexSE2(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::VectorXd>& lb,
                             const Eigen::Ref<const Eigen::VectorXd>& ub, bool fixed = false)
        : VectorVertex(values, lb, ub, fixed)
    {
    }

    // implements interface method
    void plus(int idx, double inc) override
    {
        if (idx == 2)
            _values[idx] = normalize_theta(_values[idx] + inc);
        else
            _values[idx] += inc;
    }
    // implements interface method
    void plus(const double* inc) override
    {
        assert(getDimension() >= 3);
        _values[0] += inc[0];
        _values[1] += inc[1];
        _values[2] = normalize_theta(_values[2] + inc[2]);
        if (getDimension() > 3) _values.tail(getDimension() - 3) += Eigen::Map<const Eigen::VectorXd>(inc + 3, getDimension() - 3);
    }
    // implements interface method
    void plusUnfixed(const double* inc) override { plus(inc); }

    // implements interface method
    void setData(int idx, double data) override
    {
        if (idx == 2)
            _values[idx] = normalize_theta(data);
        else
            _values[idx] = data;
    }

    //! Set values and bounds at once
    virtual void set(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::VectorXd>& lb,
                     const Eigen::Ref<const Eigen::VectorXd>& ub, bool fixed = false)
    {
        assert(values.size() == lb.size());
        assert(values.size() == ub.size());
        assert(values.size() >= 3);
        _values    = values;
        _values[2] = normalize_theta(_values[2]);
        setLowerBounds(lb);
        setUpperBounds(ub);

        setFixed(false);
    }

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief VectorVertexSE2 with support for partially fixed components
 *
 *
 * The vertex extends VectorVertexSE2 by allowing the user to
 * partially fix components of the underlying vector.
 *
 * @see VectorVertexSE2 corbo::VertexInterface corbo::VectorVertex
 *      corbo::HyperGraph corbo::EdgeInterface
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class PartiallyFixedVectorVertexSE2 : public VectorVertexSE2
{
 public:
    using Ptr  = std::shared_ptr<PartiallyFixedVectorVertexSE2>;
    using UPtr = std::unique_ptr<PartiallyFixedVectorVertexSE2>;

    //! Default constructor
    PartiallyFixedVectorVertexSE2() = default;

    explicit PartiallyFixedVectorVertexSE2(int dimension)
        : VectorVertexSE2(dimension), _fixed(Eigen::Array<bool, -1, 1>::Constant(dimension, false)), _num_unfixed(dimension)
    {
    }

    //! Construct and allocate memory for a given dimension
    explicit PartiallyFixedVectorVertexSE2(int dimension, const Eigen::Ref<const Eigen::Array<bool, -1, 1>>& fixed)
        : VectorVertexSE2(dimension), _fixed(fixed), _num_unfixed(dimension)
    {
    }

    //! Construct vertex with given values
    explicit PartiallyFixedVectorVertexSE2(const Eigen::Ref<const Eigen::VectorXd>& values)
        : VectorVertexSE2(values), _fixed(Eigen::Array<bool, -1, 1>::Constant(values.size(), false)), _num_unfixed(values.size())
    {
    }

    //! Construct vertex with given values and fixed components
    explicit PartiallyFixedVectorVertexSE2(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::Array<bool, -1, 1>>& fixed)
        : VectorVertexSE2(values), _fixed(fixed), _num_unfixed(fixed.size() - fixed.count())
    {
    }

    //! Construct vertex with given values, lower and upper bounds
    explicit PartiallyFixedVectorVertexSE2(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::VectorXd>& lb,
                                           const Eigen::Ref<const Eigen::VectorXd>& ub)
        : VectorVertexSE2(values, lb, ub), _fixed(Eigen::Array<bool, -1, 1>::Constant(values.size(), false)), _num_unfixed(values.size())
    {
    }

    // implements interface method
    int getDimensionUnfixed() const override { return _num_unfixed; }

    // implements parent method
    void setDimension(int dim) override
    {
        VectorVertexSE2::setDimension(dim);
        _fixed.setConstant(dim, false);
        _num_unfixed = dim;
    }

    //! Set values and bounds at once
    void set(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::VectorXd>& lb,
             const Eigen::Ref<const Eigen::VectorXd>& ub, bool fixed = false) override
    {
        assert(values.size() == lb.size());
        assert(values.size() == ub.size());
        assert(values.size() >= 3);
        _values    = values;
        _values[2] = normalize_theta(_values[2]);
        setLowerBounds(lb);
        setUpperBounds(ub);

        setFixed(fixed);
    }

    //! Set values and bounds at once (overload with fixed vector)
    void set(const Eigen::Ref<const Eigen::VectorXd>& values, const Eigen::Ref<const Eigen::VectorXd>& lb,
             const Eigen::Ref<const Eigen::VectorXd>& ub, const Eigen::Ref<const Eigen::Array<bool, -1, 1>>& fixed)
    {
        assert(values.size() == lb.size());
        assert(values.size() == ub.size());
        assert(values.size() >= 3);
        _values    = values;
        _values[2] = normalize_theta(_values[2]);
        setLowerBounds(lb);
        setUpperBounds(ub);

        setFixed(fixed);
    }

    //! Set component with idx (0 <= idx < dimension()) to (un)fixed
    void setFixed(int idx, bool fixed)
    {
        _fixed[idx]  = fixed;
        _num_unfixed = getDimension() - _fixed.count();
    }

    //! Set logical array [dimension() x 1] in order to fix selected components
    void setFixed(const Eigen::Ref<const Eigen::Array<bool, -1, 1>>& fixed)
    {
        _fixed       = fixed;
        _num_unfixed = getDimension() - _fixed.count();
    }

    // implements interface method
    void setFixed(bool fixed) override
    {
        _fixed.setConstant(_values.size(), fixed);
        _num_unfixed = fixed ? 0 : getDimension();
    }

    // implements interface method
    void plusUnfixed(const double* inc) override
    {
        int idx = 0;
        for (int i = 0; i < getDimension(); ++i)
        {
            if (!_fixed(i))
            {
                plus(i, inc[idx]);
                ++idx;
            }
        }
    }

    // implements interface method
    bool hasFixedComponents() const override { return _num_unfixed < getDimension(); }
    // implements interface method
    bool isFixedComponent(int idx) const override { return _fixed[idx]; }

    // implements interface method
    int getNumberFiniteLowerBounds(bool unfixed_only) const override
    {
        if (unfixed_only && _num_unfixed > 0)
        {
            int num = 0;
            for (int i = 0; i < getDimension(); ++i)
            {
                if (!_fixed[i] && _lb[i] > -corbo::CORBO_INF_DBL) num += 1;
            }
            return num;
        }
        else
            return (_lb.array() > -corbo::CORBO_INF_DBL).count();
    }

    // implements interface method
    int getNumberFiniteUpperBounds(bool unfixed_only) const override
    {
        if (unfixed_only && _num_unfixed > 0)
        {
            int num = 0;
            for (int i = 0; i < getDimension(); ++i)
            {
                if (!_fixed[i] && _ub[i] < corbo::CORBO_INF_DBL) num += 1;
            }
            return num;
        }
        else
            return (_ub.array() < corbo::CORBO_INF_DBL).count();
    }

    // implements interface method
    int getNumberFiniteBounds(bool unfixed_only) const override
    {
        if (unfixed_only && _num_unfixed > 0)
        {
            int num = 0;
            for (int i = 0; i < getDimension(); ++i)
            {
                if (!_fixed[i] && (_ub[i] < corbo::CORBO_INF_DBL || _lb[i] > -corbo::CORBO_INF_DBL)) num += 1;
            }
            return num;
        }
        else
            return (_ub.array() < corbo::CORBO_INF_DBL || _lb.array() > -corbo::CORBO_INF_DBL).count();
    }

    //! Read-only access to the underlying logical array for fixed components
    const Eigen::Array<bool, -1, 1> fixedArray() const { return _fixed; }

 protected:
    Eigen::Array<bool, -1, 1> _fixed;
    int _num_unfixed;
};

}  // namespace mpc_local_planner

#endif  // SRC_OPTIMIZATION_INCLUDE_CORBO_OPTIMIZATION_HYPER_GRAPH_VECTOR_VERTEX_H_

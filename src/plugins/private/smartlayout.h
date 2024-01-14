/*
    SPDX-FileCopyrightText: 2024 Yifan Zhu <fanzhuyifan@gmail.com>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

/**
 * @brief Adapts the algorithm from [0] to layout the windows intelligently.
 * Arranges the windows in horizontal layers, such that the widths of the layers
 * are roughly the same.
 *
 * [0] Hirschberg, Daniel S., and Lawrence L. Larmore. "The least weight
 * subsequence problem." SIAM Journal on Computing 16.4 (1987): 628-638.
 */

#pragma once

#include <QRect>

#include <vector>

class LayoutHelper;

/**
 * @brief Each Layer is a horizontal strip of windows with a maximum width and
 * height.
 */
class Layer
{
public:
    const qreal maxWidth;
    const qreal maxHeight;

    /**
     * @brief The indices of windows in this layer.
     *
     */
    const std::vector<size_t> ids;

    /**
     * @brief Initializes a new layer with the given maximum width and populates
     * it with the given windows.
     *
     * @param maxWidth The maximum width of the layer.
     * @param windowSizes The sizes of all the windows. Must be sorted in
     * ascending order by height.
     * @param windowIds Ids of the windows.
     * @param startPos windowIds[startPos] is the first window in this layer.
     * @param endPos windowIds[endPos-1] is the last window in this layer.
     */
    Layer(qreal maxWidth, const std::vector<QRectF> &windowSizes, const std::vector<size_t> &windowIds, size_t startPos, size_t endPos);

    /**
     * @brief The total width of all the windows in this layer.
     *
     */
    qreal width() const;

    /**
     * @brief The remaining width available to new windows in this layer.
     * width() + remainingWidth() == maxWidth
     *
     */
    qreal remainingWidth() const;

protected:
    qreal m_remainingWidth;
};

/**
 * @brief A LayeredPacking is a packing of windows into layers, which are
 * horizontal strips of windows.
 *
 */
class LayeredPacking
{
public:
    const qreal maxWidth;

    /**
     * @brief Construct a new LayeredPacking object from a list of windows
     * sorted by height in descending order.
     *
     * @param maxWidth The maximum width of the packing.
     * @param windowSizes must be sorted by height in ascending order
     * @param ids Ids of the windows
     * @param layerStartPos Array of indices into ids that indicate the start
     * of a new layer. Must start with 0 and end with ids.size().
     */
    LayeredPacking(qreal maxWidth, const std::vector<QRectF> &windowSizes, const std::vector<size_t> &ids, const std::vector<size_t> &layerStartPos);

    qreal width() const;
    qreal height() const;

    const std::vector<Layer> &layers() const;

protected:
    qreal m_width;
    qreal m_height;
    std::vector<Layer> m_layers;
};

/**
 * @brief The Basic algorithm from Hirschberg, Daniel S., and Lawrence L.
 * Larmore. "The least weight subsequence problem." SIAM Journal on
 * Computing 16.4 (1987): 628-638
 *
 * The Basic algorithm solves the Least Weight Subsequence Problem (LWS) for
 * concave weight functions.
 *
 * The LWS problem on the interval [a,b] is defined as follows:
 * Given a weight function weight(i,j) for all i,j in [a,b], find a subsequence
 * of [a,b], i.e. a sequence of strictly monotonically increasing indices
 * i_0 < i_2 < ... < i_t, such that the total weight,
 * sum_{k=1}^t weight(i_{k-1}, i_k), is minimized.
 *
 * A weight function is concave if for all i <= j < k <= l, the following holds:
 * weight(i,k) + weight(j,l) <= weight(i,l) + weight(j,k)
 *
 * The run time of the algorithm is O(n log n).
 *
 * Modified from the version in the paper to fix some bugs.
 *
 * @param n The length of the sequence. Solves the LWS problem on the interval
 * [0,n].
 * @param weight The weight function. Must be concave (see definition above).
 * @return std::vector<size_t> The subsequence (starting at 0 and ending at n)
 * that minimizes the total weight.
 */
std::vector<size_t> basic(size_t n, std::function<qreal(size_t, size_t)> weight);

/**
 * @brief The Bridge algorithm from Hirschberg, Daniel S., and Lawrence L.
 * Larmore. "The least weight subsequence problem." SIAM Journal on
 * Computing 16.4 (1987): 628-638
 *
 * Returns false if and only if there exists a k with @param c < k <= @param n
 * such that g( @param b, k ) < g( @param a, k ) and g( @param b, k ) < g(
 * @param c, k ). Intuitively, this returns true when b can be ignored in the
 * future because either a or c is at least as good a candidate for bestLeft.
 *
 * The input must satisfy @param a < @param b < @param c
 *
 * The run time of the algorithm is O(log n).
 *
 * @param n The length of the sequence.
 * @param g g(i, j) = f(i) + weight(i, j)
 */
bool bridge(size_t a, size_t b, size_t c, size_t n, std::function<qreal(size_t, size_t)> g);

/**
 * @brief Helper class to consolidate the high level algorithms for laying out
 * windows.
 *
 */
class LayoutHelper
{
public:
    enum PlacementMode : uint {
        Rows,
        Columns,
    };

    static void layout(const QRectF &area, const std::vector<QRectF> &windowSizes, std::vector<QRectF> &windowLayouts, enum PlacementMode mode);

protected:
    /**
     * @brief First clip @param windowSizes to be between @param minSize and
     * @param maxSize. Then add @param margins to each window size.
     */
    static void adjustSizes(const QRectF &minSize, const QRectF &maxSize, const QMarginsF &margins, std::vector<QRectF> &windowSizes);

    /**
     * @brief Use binary search to find a good packing of the @param windowSizes
     * into @param area such that the resulting packing has similar aspect ratio
     * (height/width) to @param area.
     *
     * The binary search is performed on the logarithm of the width of the
     * possible packings, and the search is terminated when the width of the
     * packing is within @param tol of the ideal width.
     *
     * We try to find a packing such that the total widths of windows in each
     * layer are close to @param idealWidthRatio times the maximum width of the
     * packing.
     *
     * In the case of identical window heights, we also try to minimize vertical
     * movement based on the @param centers of the windows.
     *
     * Run time is O(n log n log log (totalWidth / maxWidth))
     * Since we clip the window size, this is just O(n log n log log n)
     */
    static LayeredPacking
    findGoodPacking(const QRectF &area, const std::vector<QRectF> &windowSizes, const std::vector<QPointF> &centers, qreal idealWidthRatio, qreal tol);

    /**
     * @brief Output the final window layouts from the packing.
     *
     * Scale @param packing to fit @param area , remove previously added @param
     * margins, add padding and align, and store the result in @param
     * windowLayouts. In each layer, sort the windows by x coordinates of the @param centers.
     */
    static void refineAndApplyPacking(const QRectF &area, const QMarginsF &margins, LayeredPacking &packing, std::vector<QRectF> &windowLayouts, const std::vector<QPointF> &centers);

    // Reflection about the line y = x
    static QMarginsF reflect(const QMarginsF &margins)
    {
        return QMarginsF(margins.top(), margins.right(), margins.bottom(), margins.left());
    }
    static QRectF reflect(const QRectF &rect)
    {
        return QRectF(rect.y(), rect.x(), rect.height(), rect.width());
    }
    static QPointF reflect(const QPointF &point)
    {
        return point.transposed();
    }
    template<typename T>
    static std::vector<T> reflect(const std::vector<T> &v)
    {
        std::vector<T> result;
        result.reserve(v.size());
        for (const auto &x : v) {
            result.emplace_back(reflect(x));
        }
        return result;
    }
};
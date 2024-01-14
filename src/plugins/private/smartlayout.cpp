/*
    SPDX-FileCopyrightText: 2024 Yifan Zhu <fanzhuyifan@gmail.com>

    SPDX-License-Identifier: GPL-2.0-or-later
*/

#include "smartlayout.h"

#include "layoutconfig.h"

#include <deque>
#include <tuple>

Layer::Layer(qreal maxWidth, const std::vector<QRectF> &windowSizes, const std::vector<size_t> &windowIds, size_t startPos, size_t endPos)
    : maxWidth(maxWidth)
    , maxHeight(windowSizes[windowIds[endPos - 1]].height())
    , ids(windowIds.begin() + startPos, windowIds.begin() + endPos)
{
    m_remainingWidth = maxWidth;
    for (auto id = ids.begin(); id != ids.end(); ++id) {
        m_remainingWidth -= windowSizes[*id].width();
    }
}

qreal Layer::width() const
{
    return maxWidth - m_remainingWidth;
}

qreal Layer::remainingWidth() const
{
    return m_remainingWidth;
}

LayeredPacking::LayeredPacking(qreal maxWidth, const std::vector<QRectF> &windowSizes, const std::vector<size_t> &ids, const std::vector<size_t> &layerStartPos)
    : maxWidth(maxWidth)
    , m_width(0)
    , m_height(0)
{
    for (size_t i = 1; i < layerStartPos.size(); ++i) {
        m_layers.emplace_back(maxWidth, windowSizes, ids, layerStartPos[i - 1], layerStartPos[i]);
        m_width = std::max(m_width, m_layers.back().width());
        m_height += m_layers.back().maxHeight;
    }
}

qreal LayeredPacking::width() const
{
    return m_width;
}

qreal LayeredPacking::height() const
{
    return m_height;
}

const std::vector<Layer> &LayeredPacking::layers() const
{
    return m_layers;
}

std::vector<size_t> basic(size_t n, std::function<qreal(size_t, size_t)> weight)
{
    // bestLeft[j] is the second from the last element in a solution to the
    // problem restricted to [0, j]. In our case, this is where the layer
    // containing j starts
    std::vector<size_t> bestLeft(n + 1);
    // f[i] is the least weight of any subsequence starting at 0 and ending at i
    std::vector<qreal> f(n + 1);
    // d contains all current candidates for bestLeft[m]
    std::deque<size_t> d;

    f[0] = 0;

    // g(i, j) is a candidate value for f[j], and f[j] = min_{i < j} g(i, j)
    auto g = [&f, &weight](size_t i, size_t j) {
        return f[i] + weight(i, j);
    };
    d.push_back(0);
    for (size_t m = 1; m < n; ++m) {
        f[m] = g(d.front(), m);
        bestLeft[m] = d.front();

        // Modification of algorithm in paper; needed so that d.front can be correctly removed when d.size() == 1
        d.push_back(m);
        while (d.size() >= 2 && g(d[1], m + 1) <= g(d[0], m + 1)) {
            d.pop_front();
        }
        d.pop_back(); // Modification of algorithm in paper

        while (d.size() >= 2 && bridge(d[d.size() - 2], d.back(), m, n, g)) {
            d.pop_back();
        }

        // Modification of algorithm in paper; we need at least one candidate in d
        if (d.empty()) {
            d.push_back(m);
            continue;
        }

        if (g(m, n) < g(d.back(), n)) {
            d.push_back(m);
        }
    }

    // recover the solution using bestLeft
    f[n] = g(d.front(), n);
    bestLeft[n] = d.front();

    std::vector<size_t> L;
    L.push_back(n);
    size_t m = n;

    while (m > 0) {
        m = bestLeft[m];
        L.push_back(m);
    }

    return std::vector<size_t>(L.rbegin(), L.rend());
}

bool bridge(size_t a, size_t b, size_t c, size_t n, std::function<qreal(size_t, size_t)> g)
{
    if (c == n) {
        return true;
    }
    if (g(a, n) <= g(b, n)) {
        return true;
    }
    size_t low = c;
    size_t high = n;
    while (high - low >= 2) {
        size_t mid = (low + high) / 2;
        if (g(a, mid) <= g(b, mid)) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return (g(c, high) <= g(b, high));
}

void smartPacking(qreal maxWidth, qreal idealWidth, const std::vector<size_t> &ids, const std::vector<qreal> &cumWidths, std::vector<size_t> &layerStartPos)
{
    /**
     * The weight function is designed such that
     * 1. The weight function is concave (see definition in basic())
     * 2. It scales like (width - idealWidth) ^ 2 for width < idealWidth
     * 3. Exceeding maxWidth is guaranteed to be worse than any other solution
     *
     * 1. holds as long as weight(i, j) = f(cumWidths[j] - cumWidths[i]) for some convex function f
     * 3. is guaranteed by making the penalty of exceeding maxWidth at least
     * cumWidths.size(), which strictly upper bounds the total weight of placing
     * each window in its own layer
     */
    auto weight = [maxWidth, idealWidth, &cumWidths](size_t i, size_t j) {
        qreal width = cumWidths[j] - cumWidths[i];
        if (width < idealWidth) {
            return (width - idealWidth) * (width - idealWidth) / maxWidth / maxWidth;
        }
        return cumWidths.size() * (width - idealWidth) / (maxWidth - idealWidth);
    };

    layerStartPos = basic(ids.size(), weight);
}

void LayoutHelper::layout(const QRectF &area, const std::vector<QRectF> &windowSizes, std::vector<QRectF> &windowLayouts, enum PlacementMode mode)
{
    const auto config = LayoutConfig::self();

    const qreal shortSide = std::min(area.width(), area.height());
    const QMarginsF margins(shortSide * config->relativeMarginLeft(),
                            shortSide * config->relativeMarginTop(),
                            shortSide * config->relativeMarginRight(),
                            shortSide * config->relativeMarginBottom());
    const qreal minLength = config->relativeMinLength() * shortSide;
    const QRectF minSize = QRectF(0, 0, minLength, minLength);

    std::vector<QPointF> centers;
    for (const QRectF &windowSize : windowSizes) {
        centers.push_back(windowSize.center());
    }

    windowLayouts.clear();

    for (const QRectF &windowSize : windowSizes) {
        windowLayouts.push_back(windowSize);
    }

    // windows bigger than 4x the area are considered ill-behaved and their sizes are clipped
    adjustSizes(minSize, QRectF(0, 0, 4 * area.width(), 4 * area.height()), margins, windowLayouts);

    if (mode == PlacementMode::Rows) {
        LayeredPacking bestPacking = findGoodPacking(area, windowLayouts, centers, config->idealWidthRatio(), config->searchTolerance());
        refineAndApplyPacking(area, margins, bestPacking, windowLayouts, centers);
    } else {
        std::vector<QRectF> windowLayoutsReflected(reflect(windowLayouts));
        std::vector<QPointF> centersReflected(reflect(centers));

        LayeredPacking bestPacking = findGoodPacking(area.transposed(), windowLayoutsReflected, centersReflected, config->idealWidthRatio(), config->searchTolerance());
        refineAndApplyPacking(area.transposed(), reflect(margins), bestPacking, windowLayoutsReflected, centersReflected);

        windowLayouts = reflect(windowLayoutsReflected);
    }
}

void LayoutHelper::adjustSizes(const QRectF &minSize, const QRectF &maxSize, const QMarginsF &margins, std::vector<QRectF> &windowSizes)
{
    for (QRectF &windowSize : windowSizes) {
        windowSize.setWidth(std::clamp(windowSize.width(), minSize.width(), maxSize.width()));
        windowSize.setHeight(std::clamp(windowSize.height(), minSize.height(), maxSize.height()));
        windowSize += margins;
    }
}

LayeredPacking
LayoutHelper::findGoodPacking(const QRectF &area, const std::vector<QRectF> &windowSizes, const std::vector<QPointF> &centers, qreal idealWidthRatio, qreal tol)
{
    std::vector<std::tuple<size_t, QRectF, QPointF>> windowSizesWithIds;

    for (size_t i = 0; i < windowSizes.size(); ++i) {
        windowSizesWithIds.emplace_back(i, windowSizes[i], centers[i]);
    }

    std::stable_sort(
        windowSizesWithIds.begin(), windowSizesWithIds.end(),
        [](const auto &a, const auto &b) {
            // in case of same height, sort by y to minimize vertical movement
            return std::tuple(std::get<1>(a).height(), std::get<2>(a).y())
                < std::tuple(std::get<1>(b).height(), std::get<2>(b).y());
        });

    std::vector<size_t> ids;
    std::vector<qreal> cumWidths;

    /**
     * Minimum and maximum strip widths to use in the binary search.
     * Strips should be at least as wide as the widest window, and at most as
     * wide as the sum of all window widths.
     */
    qreal stripWidthMin = 0;
    qreal stripWidthMax = 0;

    cumWidths.push_back(0);
    for (const auto &windowSizeWithId : windowSizesWithIds) {
        ids.push_back(std::get<0>(windowSizeWithId));
        qreal width = std::get<1>(windowSizeWithId).width();
        cumWidths.push_back(cumWidths.back() + width);

        stripWidthMin = std::max(stripWidthMin, width);
        stripWidthMax += width;
    }

    qreal targetRatio = area.height() / area.width();

    auto findPacking = [&windowSizes, &ids, &cumWidths, idealWidthRatio](qreal stripWidth) {
        std::vector<size_t> layerStartPos;
        smartPacking(stripWidth, stripWidth * idealWidthRatio, ids, cumWidths, layerStartPos);
        LayeredPacking result(stripWidth, windowSizes, ids, layerStartPos);
        assert(result.width() <= stripWidth);
        return result;
    };
    stripWidthMax /= idealWidthRatio;

    // the placement with the minimum strip width corresponds with a big aspect
    // ratio (ratioHigh), and the placement with the maximum strip width
    // corresponds with a small aspect ratio (ratioLow)

    std::unique_ptr<LayeredPacking> placementWidthMin = std::make_unique<LayeredPacking>(findPacking(stripWidthMin));
    qreal ratioHigh = placementWidthMin->height() / placementWidthMin->width();

    if (ratioHigh <= targetRatio) {
        return *placementWidthMin;
    }

    std::unique_ptr<LayeredPacking> placementWidthMax = std::make_unique<LayeredPacking>(findPacking(stripWidthMax));
    qreal ratioLow = placementWidthMax->height() / placementWidthMax->width();

    if (ratioLow >= targetRatio) {
        return *placementWidthMax;
    }

    while (stripWidthMax / stripWidthMin > 1 + tol) {
        qreal stripWidthMid = std::sqrt(stripWidthMin * stripWidthMax);
        std::unique_ptr<LayeredPacking> placementMid = std::make_unique<LayeredPacking>(findPacking(stripWidthMid));
        qreal ratioMid = placementMid->height() / placementMid->width();

        if (ratioMid > targetRatio) {
            stripWidthMin = stripWidthMid;
            placementWidthMin = std::move(placementMid);
            ratioHigh = ratioMid;
        } else {
            // small optimization: use the actual strip width
            stripWidthMax = placementMid->width();
            placementWidthMax = std::move(placementMid);
            ratioLow = ratioMid;
        }
    }

    if (ratioHigh - targetRatio < targetRatio - ratioLow) {
        return *placementWidthMin;
    } else {
        return *placementWidthMax;
    }
}

void LayoutHelper::refineAndApplyPacking(const QRectF &area, const QMarginsF &margins, LayeredPacking &packing, std::vector<QRectF> &windowLayouts, const std::vector<QPointF> &centers)
{
    const auto config = LayoutConfig::self();

    // Scale packing to fit area
    qreal scale = std::min(area.width() / packing.width(), area.height() / packing.height());
    scale = std::min(scale, config->maxScale());

    const QMarginsF scaledMargins =
        QMarginsF(margins.left() * scale, margins.top() * scale,
                  margins.right() * scale, margins.bottom() * scale);
    qreal y = area.y();

    // The maximum gap in additional to margins to leave between windows
    qreal maxGapY = config->maxGapRatio() * (scaledMargins.top() + scaledMargins.bottom());
    qreal maxGapX = config->maxGapRatio() * (scaledMargins.left() + scaledMargins.right());

    // center align y
    qreal extraY = area.height() - packing.height() * scale;
    qreal gapY = std::min(maxGapY, extraY / (packing.layers().size() + 1));
    y += (extraY - gapY * (packing.layers().size() - 1)) / 2;

    // smaller windows "float" to the top
    for (const auto &layer : packing.layers()) {
        qreal extraX = area.width() - layer.width() * scale;
        qreal gapX = std::min(maxGapX, extraX / (layer.ids.size() + 1));
        qreal x = area.x();
        x += (extraX - gapX * (layer.ids.size() - 1)) / 2;

        std::vector<size_t> ids(layer.ids);
        std::stable_sort(ids.begin(), ids.end(), [&centers](size_t a, size_t b) {
            return centers[a].x() < centers[b].x();
        });
        for (auto id : ids) {
            QRectF &windowLayout = windowLayouts[id];
            // center align
            qreal newY = y + (layer.maxHeight - windowLayout.height()) * scale / 2;
            windowLayout = QRectF(x, newY, windowLayout.width() * scale, windowLayout.height() * scale);

            x += windowLayout.width();
            x += gapX;

            windowLayout -= scaledMargins;
        }

        y += layer.maxHeight * scale;
        y += gapY;
    }
}

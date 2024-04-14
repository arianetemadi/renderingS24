/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/block.h>
#include <nori/bitmap.h>
#include <nori/rfilter.h>
#include <nori/bbox.h>
#include <tbb/tbb.h>

NORI_NAMESPACE_BEGIN

ImageBlock::ImageBlock(const Vector2i &size, const ReconstructionFilter *filter) 
        : m_offset(0, 0), m_size(size) {
    if (filter) {
        /* Tabulate the image reconstruction filter for performance reasons */
        m_filterRadius = filter->getRadius();
        m_borderSize = (int) std::ceil(m_filterRadius - 0.5f);
        m_filter = new float[NORI_FILTER_RESOLUTION + 1];
        for (int i=0; i<NORI_FILTER_RESOLUTION; ++i) {
            float pos = (m_filterRadius * i) / NORI_FILTER_RESOLUTION;
            m_filter[i] = filter->eval(pos);
        }
        m_filter[NORI_FILTER_RESOLUTION] = 0.0f;
        m_lookupFactor = NORI_FILTER_RESOLUTION / m_filterRadius;
        int weightSize = (int) std::ceil(2*m_filterRadius) + 1;
        m_weightsX = new float[weightSize];
        m_weightsY = new float[weightSize];
        memset(m_weightsX, 0, sizeof(float) * weightSize);
        memset(m_weightsY, 0, sizeof(float) * weightSize);
    }

    /* Allocate space for pixels and border regions */
    resize(size.y() + 2*m_borderSize, size.x() + 2*m_borderSize);
}

ImageBlock::~ImageBlock() {
    delete[] m_filter;
    delete[] m_weightsX;
    delete[] m_weightsY;
}

Bitmap *ImageBlock::toBitmap() const {
    Bitmap *result = new Bitmap(m_size);
    for (int y=0; y<m_size.y(); ++y)
        for (int x=0; x<m_size.x(); ++x)
			result->coeffRef(y, x) = coeff(y + m_borderSize, x + m_borderSize).divideByFilterWeight();
    return result;
}

void ImageBlock::fromBitmap(const Bitmap &bitmap) {
    if (bitmap.cols() != cols() || bitmap.rows() != rows())
        throw NoriException("Invalid bitmap dimensions!");

    for (int y=0; y<m_size.y(); ++y)
        for (int x=0; x<m_size.x(); ++x)
            coeffRef(y, x) << bitmap.coeff(y, x), 1;
}

void ImageBlock::put(const Point2f &_pos, const Color3f &value) {
    if (!value.isValid()) {
        /* If this happens, go fix your code instead of removing this warning ;) */
        cerr << "Integrator: computed an invalid radiance value: " << value.toString() << endl;
        return;
    }
    Index y = std::max(Index(0), std::min(rows()-1, Index(_pos.y() - m_offset.y())));
    Index x = std::max(Index(0), std::min(cols()-1, Index(_pos.x() - m_offset.x())));
    coeffRef(y, x) += Color4f(value);    // Color4f appends a 1 in the constructor taking a vec3
}
    
void ImageBlock::put(ImageBlock &b) {
    Vector2i offset = b.getOffset() - m_offset +
        Vector2i::Constant(m_borderSize - b.getBorderSize());
    Vector2i size   = b.getSize()   + Vector2i(2*b.getBorderSize());

    tbb::mutex::scoped_lock lock(m_mutex);

    block(offset.y(), offset.x(), size.y(), size.x()) 
        += b.topLeftCorner(size.y(), size.x());
}

std::string ImageBlock::toString() const {
    return tfm::format("ImageBlock[offset=%s, size=%s]]",
        m_offset.toString(), m_size.toString());
}

BlockGenerator::BlockGenerator(const Vector2i &size, int blockSize)
        : m_size(size), m_blockSize(blockSize) {
    m_numBlocks = Vector2i(
        (int) std::ceil(size.x() / (float) blockSize),
        (int) std::ceil(size.y() / (float) blockSize));
    m_blocksLeft = m_numBlocks.x() * m_numBlocks.y();
    m_direction = ERight;
    m_block = Point2i(m_numBlocks / 2);
    m_stepsLeft = 1;
    m_numSteps = 1;
}

bool BlockGenerator::next(ImageBlock &block) {
    tbb::mutex::scoped_lock lock(m_mutex);

    if (m_blocksLeft == 0)
        return false;

    Point2i pos = m_block * m_blockSize;
    block.setOffset(pos);
    block.setSize((m_size - pos).cwiseMin(Vector2i::Constant(m_blockSize)));

    if (--m_blocksLeft == 0)
        return true;

    do {
        switch (m_direction) {
            case ERight: ++m_block.x(); break;
            case EDown:  ++m_block.y(); break;
            case ELeft:  --m_block.x(); break;
            case EUp:    --m_block.y(); break;
        }

        if (--m_stepsLeft == 0) {
            m_direction = (m_direction + 1) % 4;
            if (m_direction == ELeft || m_direction == ERight) 
                ++m_numSteps;
            m_stepsLeft = m_numSteps;
        }
    } while ((m_block.array() < 0).any() ||
             (m_block.array() >= m_numBlocks.array()).any());

    return true;
}

NORI_NAMESPACE_END

#include "Huffman.h"

#include<cstring>
#include <istream>
#include<memory>
#include <ostream>
#include<queue>
#include<vector>

using namespace Arch;

const size_t SIZE_OF_BLOCK = 1024;  // 1 KiB
const size_t CHAR_CAPACITY = (1 << 8);
const size_t BITS_IN_CHAR = 8;

enum Error {
    OK = 0,
    WRONG_ARGUMENTS,
    CANT_DECODE_BPT,
    WRONG_PREFIX_TREE,
    WRONG_DATA,
};

bool check_bit(size_t x, size_t bit) {
    return x & (1 << bit);
}

uint64_t readText(std::istream *in, uint64_t freqs[]) {
    char block[SIZE_OF_BLOCK];
    uint64_t text_sz = 0;
    while (true) {
        in->read(block, SIZE_OF_BLOCK);
        size_t real_sz = in->gcount();
        text_sz += real_sz;
        for (size_t i = 0; i < real_sz; ++i) {
            ++freqs[static_cast<uint8_t>(block[i])];
        }
        if (real_sz != SIZE_OF_BLOCK) {
            break;
        }
    }
    return text_sz;
}

class BinaryPrefixTree;

class Node {
private:
    std::shared_ptr<Node> lson = nullptr, rson = nullptr;
    uint8_t ch;

public:
    Node() = default;
    Node(uint8_t ch_new) noexcept : ch(ch_new) {}
    Node(std::shared_ptr<Node> lson_new, std::shared_ptr<Node> rson_new) noexcept :
                        lson(lson_new), rson(rson_new) {}

    bool isTerm() const noexcept {
        return (!lson && !rson);
    }

    uint8_t getChar() const noexcept {
        return ch;
    }

    bool operator<(const Node& other) const noexcept {
        return ch < other.ch;
    }

    friend class BinaryPrefixTree;
};

class CodingSystem {
private:
    uint8_t lengths[CHAR_CAPACITY];
    uint32_t codes[CHAR_CAPACITY];

public:
    CodingSystem() = default;
    CodingSystem(const CodingSystem& other) noexcept {
        for (size_t i = 0; i < CHAR_CAPACITY; ++i) {
            lengths[i] = other.lengths[i];
            codes[i] = other.codes[i];
        }
    }
    CodingSystem(CodingSystem&& other) noexcept {
        std::swap(other.codes, codes);
        std::swap(other.lengths, lengths);
    }

    void SetCode(uint8_t new_ch, uint8_t l, uint32_t code) noexcept {
        lengths[static_cast<size_t>(new_ch)] = l;
        codes[static_cast<size_t>(new_ch)] = code;
    }

    std::pair<uint8_t, uint32_t> getCode(uint8_t new_ch) const noexcept {
        return {lengths[static_cast<size_t>(new_ch)], codes[static_cast<size_t>(new_ch)]};
    }
};

class BinaryPrefixTree {
private:
    std::shared_ptr<Node> root;

public:
    BinaryPrefixTree() noexcept : root(new Node()) {}
    BinaryPrefixTree(std::shared_ptr<Node> root_new) noexcept : root(root_new) {}
    BinaryPrefixTree(BinaryPrefixTree&& other) noexcept {
        root = other.root;
        other.root = nullptr;
    }

    void Fill(CodingSystem& to_fill,
              std::shared_ptr<Node> v,
              uint32_t curcode, uint8_t len) const noexcept {
        if (v->isTerm()) {
            to_fill.SetCode(v->ch, len, curcode);
            return;
        }
        if (v->lson) {
            Fill(to_fill, v->lson, (curcode << 1), len + 1);
        }
        if (v->rson) {
            Fill(to_fill, v->rson, (curcode << 1) + 1, len + 1);
        }
    }

    void Fill(CodingSystem& to_fill) const noexcept {
        Fill(to_fill, root, 0, 0);
    }

    bool Insert(uint32_t code, uint8_t len, uint8_t new_char, std::shared_ptr<Node> v) {
        // returns true if new node is a leave, otherwise - false
        if (len == 0) {
            v->ch = new_char;
            return false;
        }
        bool certainly_leave = false;
        if (check_bit(code, len - 1)) {
            if (!v->rson) {
                v->rson.reset(new Node());
                certainly_leave = true;
            }
            return (Insert(code, len - 1, new_char, v->rson) || certainly_leave);
        } else {
            if (!v->lson) {
                v->lson.reset(new Node());
                certainly_leave = true;
            }
            return (Insert(code, len - 1, new_char, v->lson) || certainly_leave);
        }
    }

    bool Insert(uint32_t code, uint8_t len, uint8_t new_char) {
        return Insert(code, len, new_char, root);
    }

    int readChars(std::istream *in, std::ostream *out, uint64_t text_sz) {
        uint8_t lastbyte = 0;
        int index = -1;
        while (text_sz) {
            std::shared_ptr<Node> curv(root);
            while (!curv->isTerm()) {
                if (index == -1) {
                    in->read(reinterpret_cast<char*>(&lastbyte), sizeof(uint8_t));
                    if (!in->gcount()) {
                        return Error::WRONG_DATA;
                    }
                    index = BITS_IN_CHAR - 1;
                }
                if (check_bit(lastbyte, index)) {
                    curv = curv->rson;
                } else {
                    curv = curv->lson;
                }
                --index;
                if (!curv) {
                    return Error::WRONG_DATA;
                }
            }
            char remem = static_cast<char>(curv->ch);
            out->write(&remem, sizeof(char));
            --text_sz;
        }
        in->read(reinterpret_cast<char*>(&lastbyte), sizeof(uint8_t));
        if (in->gcount()) {
            return Error::WRONG_DATA;
        } else {
            return Error::OK;
        }
    }
};

struct NodePtrHelper {
    uint64_t freq = 0;
    std::shared_ptr<Node> node_ptr;
    size_t depth = 0;

    bool operator<(const NodePtrHelper& other) const noexcept {
        if (freq != other.freq)
            return freq > other.freq;
        return depth > other.depth;
    }
};

BinaryPrefixTree buildBinaryPrefixTree(uint64_t freqs[]) {
    std::priority_queue<NodePtrHelper> queue;
    for (size_t i = 0; i < CHAR_CAPACITY; ++i) {
        queue.push(NodePtrHelper{freqs[i],
                        std::shared_ptr<Node>(new Node(static_cast<uint8_t>(i))), 0});
    }
    while (queue.size() > 1) {
        auto top1 = queue.top();
        queue.pop();
        auto top2 = queue.top();
        queue.pop();
        std::shared_ptr<Node> new_node_ptr(new Node(top1.node_ptr, top2.node_ptr));
        queue.push(NodePtrHelper{top1.freq + top2.freq, new_node_ptr,
                    std::max(top1.depth, top2.depth) + 1});
    }
    return BinaryPrefixTree(queue.top().node_ptr);
}

void encode(std::istream *in,
            std::ostream *out,
            const CodingSystem& table_of_codes,
            uint64_t text_sz) {
    out->write(reinterpret_cast<const char*>(&text_sz), sizeof(uint64_t));
    for (size_t i = 0; i < CHAR_CAPACITY; ++i) {
        auto [len, code] = table_of_codes.getCode(i);
        out->write(reinterpret_cast<const char*>(&len), sizeof(uint8_t));
        out->write(reinterpret_cast<const char*>(&code), sizeof(uint32_t));
    }
    if (text_sz == 0) {
        return;
    }
    uint8_t constructed_byte = 0;
    int filled = BITS_IN_CHAR - 1;
    char block[SIZE_OF_BLOCK];
    while (true) {
        in->read(block, SIZE_OF_BLOCK);
        size_t real_sz = in->gcount();
        for (size_t i = 0; i < real_sz; ++i) {
            auto [len, code] = table_of_codes.getCode(static_cast<uint8_t>(block[i]));
            for (int ind = len - 1; ind >= 0; --ind) {
                if (filled == -1) {
                    filled = BITS_IN_CHAR - 1;
                    out->write(reinterpret_cast<const char*>(&constructed_byte), sizeof(uint8_t));
                    constructed_byte = 0;
                }
                if (check_bit(code, ind)) {
                    constructed_byte += (1 << filled);
                }
                --filled;
            }
        }
        if (real_sz != SIZE_OF_BLOCK) {
            break;
        }
    }
    out->write(reinterpret_cast<const char*>(&constructed_byte), sizeof(uint8_t));
}

int Arch::compress(std::istream *in, std::ostream *out) {
    if (in == nullptr || out == nullptr) {
        return Error::WRONG_ARGUMENTS;
    }
    uint64_t frequencies[CHAR_CAPACITY];
    std::memset(frequencies, 0, sizeof frequencies);
    uint64_t text_sz = readText(in, frequencies);
    CodingSystem table_of_codes;
    BinaryPrefixTree coding_bpt(buildBinaryPrefixTree(frequencies));
    coding_bpt.Fill(table_of_codes);
    in->clear();
    in->seekg(0, std::ios_base::beg);
    encode(in, out, table_of_codes, text_sz);
    return Error::OK;
}

int extractBPT(std::istream *in, BinaryPrefixTree& tree) {
    for (size_t i = 0; i < CHAR_CAPACITY; ++i) {
        uint8_t len;
        uint32_t code;
        in->read(reinterpret_cast<char*>(&len), sizeof(uint8_t));
        if (!in->gcount()) {
            return Error::CANT_DECODE_BPT;
        }
        in->read(reinterpret_cast<char*>(&code), sizeof(uint32_t));
        if (in->gcount() != sizeof(uint32_t)) {
            return Error::CANT_DECODE_BPT;
        }
        if (!tree.Insert(code, len, static_cast<uint8_t>(i))) {
            return Error::WRONG_PREFIX_TREE;
        }
    }
    return Error::OK;
}

int decode(std::istream *in, std::ostream *out, BinaryPrefixTree& tree, uint64_t text_sz) {
    return tree.readChars(in, out, text_sz);
}

int Arch::decompress(std::istream *in, std::ostream *out) {
    if (in == nullptr || out == nullptr) {
        return Error::WRONG_ARGUMENTS;
    }
    uint64_t text_sz;
    in->read(reinterpret_cast<char*>(&text_sz), sizeof(uint64_t));
    if (in->gcount() != sizeof(uint64_t)) {
        return Error::WRONG_DATA;
    }
    BinaryPrefixTree tree;
    int run_result = extractBPT(in, tree);
    if (run_result != Error::OK) {
        return run_result;
    }
    return decode(in, out, tree, text_sz);
}
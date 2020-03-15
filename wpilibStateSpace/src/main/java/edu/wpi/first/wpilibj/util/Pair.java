package edu.wpi.first.wpilibj.util;

public class Pair<A, B> {
    private final A m_first;
    private final B m_second;

    public Pair(A first, B second) {
        m_first = first;
        m_second = second;
    }

    public A getFirst() {
        return m_first;
    }

    public B getSecond() {
        return m_second;
    }
}
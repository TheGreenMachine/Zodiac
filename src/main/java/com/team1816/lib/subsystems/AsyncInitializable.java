package com.team1816.lib.subsystems;

import java.util.Arrays;
import java.util.concurrent.CompletableFuture;
import java.util.stream.Stream;

public interface AsyncInitializable {
    CompletableFuture<Void> initAsync();

    static CompletableFuture<Void> join(Stream<? extends AsyncInitializable> initializables) {
        return CompletableFuture.allOf(
            initializables
                .map(AsyncInitializable::initAsync)
                .toArray(CompletableFuture[]::new)
        );
    }
}

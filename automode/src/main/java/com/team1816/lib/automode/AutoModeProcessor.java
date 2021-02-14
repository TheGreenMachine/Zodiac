package com.team1816.lib.automode;

import javax.annotation.processing.AbstractProcessor;
import javax.annotation.processing.RoundEnvironment;
import javax.annotation.processing.SupportedAnnotationTypes;
import javax.annotation.processing.SupportedSourceVersion;
import javax.lang.model.SourceVersion;
import javax.lang.model.element.Element;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.ElementFilter;
import javax.tools.JavaFileObject;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Set;

@SupportedAnnotationTypes("com.team1816.lib.automode.AutoMode")
@SupportedSourceVersion(SourceVersion.RELEASE_11)
public class AutoModeProcessor extends AbstractProcessor {
    @Override
    public boolean process(Set<? extends TypeElement> annotations, RoundEnvironment roundEnv) {
        Collection<? extends Element> annotatedElements = roundEnv.getElementsAnnotatedWith(AutoMode.class);
        List<TypeElement> types = ElementFilter.typesIn(annotatedElements);

        StringBuilder builder = new StringBuilder()
            .append("package com.team1816.frc2020.auto;\n\n")
            .append("import com.team1816.lib.auto.modes.AutoModeBase;\n")
            .append("import java.util.function.Supplier;\n\n")
            .append("public enum AutoModes {\n");

        for (int i = 0; i < types.size(); i++) {
            var type = types.get(i);
            var name = type.getAnnotation(AutoMode.class).value();
            var className = type.getQualifiedName();
            builder.append("    " + enumCase(name) + "(" + className + "::new, \"" + name + "\")");
            if (i != types.size() - 1) {
                builder.append(",");
            } else {
                builder.append(";");
            }
            builder.append("\n");
        }

        builder
            .append("\n")
            .append("private final Supplier<AutoModeBase> constructor;\n")
            .append("private final String name;\n")
            .append("private AutoModes(Supplier<AutoModeBase> c, String n) {\n")
            .append("    constructor = c;\n")
            .append("    name = n;\n}\n")
            .append("public Supplier<AutoModeBase> getConstructor() {\n")
            .append("    return constructor;\n")
            .append("}\n\n")
            .append("public String getName() { return name; }\n\n")
            .append("}\n");

        try {
            JavaFileObject javaFileObject = processingEnv
                .getFiler()
                .createSourceFile("com.team1816.frc2020.auto.AutoModes");
            var writer = javaFileObject.openWriter();
            writer.write(builder.toString());
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return false;
    }

    private static String enumCase(String input) {
        return input.replaceAll(" ", "_").toUpperCase();
    }
}
